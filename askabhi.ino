#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include "time.h"
#include <math.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <HTTPClient.h>

// ------------------- CONFIGURATION -------------------
#define WIFI_SSID     "Ajay Sabarigireessh K P"
#define WIFI_PASSWORD "Askabhi@5909"

#define BUZZER_PIN       27
#define BUZZER_CHANNEL   0
#define PIN_LED          2
#define BUTTON_PIN       14
#define SDA_PIN          21
#define SCL_PIN          22

const float fallThreshold = 300000;
const int sampleInterval = 10;
const unsigned long CRITICAL_TIMEOUT = 5000;  // 5 sec
const unsigned long DOUBLE_PRESS_TIMEOUT = 1000; // 1 sec

const char* ntpServer = "in.pool.ntp.org";
const long gmtOffset_sec = 19800;
const int daylightOffset_sec = 0;

// ------------------- API -------------------
const char* heartRateUrl = "http://13.233.118.161:3000/api/heart-rate/live";
const char* fallUrl      = "http://13.233.118.161:3000/api/fall-detection";

String token = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJ1c2VySWQiOjIsImVtYWlsIjoiamVldmFuQGV4YW1wbGUuY29tIiwibmFtZSI6IkplZXZhbiIsInJvbGUiOiJ1c2VyIiwiaWF0IjoxNzU2MTA4MTEyLCJleHAiOjE3NTYxMTE3MTJ9.SrWXTM9eF9IOLNgoCzaleoL6-1BIA2zsetst7d-bCwo";  // Replace with actual JWT token
int userId = 2;

// ------------------- VARIABLES -------------------
char dateTimeStr[30];
float prevAccX = 0.0, prevAccY = 0.0, prevAccZ = 0.0;
bool buzzerActive = false;
bool criticalTriggered = false;
unsigned long fallStartTime = 0;

MPU6050 mpu;

// Button press tracking
unsigned long lastButtonPressTime = 0;
bool waitingForSecondPress = false;

// ------------------- GPS -------------------
HardwareSerial neogps(1);
TinyGPSPlus gps;
#define RXD2 16
#define TXD2 17
double gpsLat = 0.0, gpsLng = 0.0;

// ------------------- HEART RATE -------------------
MAX30105 particleSensor;
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute = 0;
int beatAvg = 0;

// ------------------- FUNCTIONS -------------------
void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("[Warning] Failed to obtain time");
    return;
  }
  strftime(dateTimeStr, sizeof(dateTimeStr), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  Serial.print("[Time] Current Time: ");
  Serial.println(dateTimeStr);
}

// Send Heart Rate to API
void sendHeartRate(int heartRate) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(heartRateUrl);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " + token);

    String jsonBody = "{\"userId\": " + String(userId) + ", \"heartRate\": " + String(heartRate) + "}";
    int httpResponseCode = http.POST(jsonBody);

    Serial.print("[HTTP] HeartRate Response: ");
    Serial.println(httpResponseCode);
    http.end();
  }
}

// Send Fall Detection to API
void sendFallDetection(String direction, String severity, double lat, double lng) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(fallUrl);
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", "Bearer " + token);

    String jsonBody = "{\"userId\": " + String(userId) +
                      ", \"direction\": \"" + direction +
                      "\", \"severity\": \"" + severity +
                      "\", \"latitude\": " + String(lat, 6) +
                      ", \"longitude\": " + String(lng, 6) + "}";

    int httpResponseCode = http.POST(jsonBody);

    Serial.print("[HTTP] Fall Detection Response: ");
    Serial.println(httpResponseCode);
    http.end();
  }
}

// ------------------- SETUP -------------------
void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.initialize();
  Serial.begin(115200);

  neogps.begin(9600, SERIAL_8N1, RXD2, TXD2);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  ledcSetup(BUZZER_CHANNEL, 2000, 16);
  ledcAttachPin(BUZZER_PIN, BUZZER_CHANNEL);

  Serial.println(mpu.testConnection() ? "MPU6050 connected" : "[Error] MPU6050 connection failed");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("[WiFi] Connecting");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" Connected!");
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
  digitalWrite(PIN_LED, HIGH);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("[Error] MAX30102 not found. Check wiring.");
    while (1);
  }
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  Serial.println("MAX30102 Initialized. Place your finger.");
}

// ------------------- LOOP -------------------
void loop() {
  static unsigned long lastTime = 0;

  // ========== FALL DETECTION ==========
  if (millis() - lastTime >= sampleInterval) {
    lastTime = millis();
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    float accX = ax / 9.8;
    float accY = ay / 9.8;
    float accZ = az / 9.8;

    float jerkX = (accX - prevAccX) / (sampleInterval / 1000.0);
    float jerkY = (accY - prevAccY) / (sampleInterval / 1000.0);
    float jerkZ = (accZ - prevAccZ) / (sampleInterval / 1000.0);
    float jerkMagnitude = sqrt(jerkX * jerkX + jerkY * jerkY + jerkZ * jerkZ);

    prevAccX = accX;
    prevAccY = accY;
    prevAccZ = accZ;

    if (jerkMagnitude > fallThreshold && !buzzerActive) {
      Serial.println("[ALERT] Fall Detected!");
      ledcWriteTone(BUZZER_CHANNEL, 5000);
      buzzerActive = true;
      fallStartTime = millis();
      criticalTriggered = false;
    }
  }

  // Button Check
  if (digitalRead(BUTTON_PIN) == LOW) {
    delay(50);
    if (digitalRead(BUTTON_PIN) == LOW) {
      unsigned long now = millis();
      if (buzzerActive) {
        ledcWrite(BUZZER_CHANNEL, 0);
        buzzerActive = false;
        lastButtonPressTime = now;
        waitingForSecondPress = true;
      } else if (waitingForSecondPress && (now - lastButtonPressTime <= DOUBLE_PRESS_TIMEOUT)) {
        waitingForSecondPress = false;
      } else {
        waitingForSecondPress = false;
      }
      while (digitalRead(BUTTON_PIN) == LOW);
    }
  }

  if (waitingForSecondPress && millis() - lastButtonPressTime > DOUBLE_PRESS_TIMEOUT) {
    waitingForSecondPress = false;
  }

  // Critical Fall
  if (buzzerActive && !criticalTriggered && (millis() - fallStartTime > CRITICAL_TIMEOUT)) {
    ledcWrite(BUZZER_CHANNEL, 0);
    buzzerActive = false;
    criticalTriggered = true;

    bool gotLocation = false;
    unsigned long gpsStart = millis();
    while (millis() - gpsStart < 5000) {
      while (neogps.available() > 0) {
        gps.encode(neogps.read());
        if (gps.location.isUpdated()) {
          gpsLat = gps.location.lat();
          gpsLng = gps.location.lng();
          gotLocation = true;
          break;
        }
      }
      if (gotLocation) break;
    }

    Serial.print("[GPS] Lat: ");
    Serial.print(gpsLat, 6);
    Serial.print(" Lng: ");
    Serial.println(gpsLng, 6);

    sendFallDetection("forward", "critical", gpsLat, gpsLng);
  }

  // Heart Rate Reading
  long irValue = particleSensor.getIR();
  if (irValue > 50000) {
    if (checkForBeat(irValue)) {
      long delta = millis() - lastBeat;
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0);

      if (beatsPerMinute < 255 && beatsPerMinute > 20) {
        rates[rateSpot++] = (byte)beatsPerMinute;
        rateSpot %= RATE_SIZE;

        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++) beatAvg += rates[x];
        beatAvg /= RATE_SIZE;

        if (beatAvg > 0) {
          Serial.print("HEART RATE BPM = ");
          Serial.println(beatAvg);
          sendHeartRate(beatAvg);
        }
      }
    }
  }

  delay(20);
}