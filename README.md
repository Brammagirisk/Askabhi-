
# Askabhi - ESP32 Health Monitoring & Fall Detection System 💓📘 Overview

With an aging population, ensuring the safety of elderly people, disabled individuals, and children is crucial. This IoT wearable health and safety monitoring system utilizes the ESP32 along with sensors like MAX30102, MPU6050, and Neo-6M GPS. The system continuously measures heart rate, detects falls, and sends GPS-enabled alerts over Wi-Fi to a remote API.

---

# ⚙️ Features
✅ Heart Rate Monitoring with MAX30102 (IR & Red LED optical sensor)

✅ Fall Detection using MPU6050 accelerometer & jerk 

✅ GPS Location Tracking via Neo-6M module

✅ Emergency Buzzer Alert

✅ Double-Press Button Cancel to avoid false alarms

✅ Real-Time Clock synchronization using NTP

✅ Wi-Fi and HTTP API communication

✅ JWT Token Authentication for secure uploads

---

### 🧠 Working PrincipleHeart Rate Measurement

MAX30102 captures IR signals.

Heartbeats detected via checkForBeat() function.

Average BPM sent to /api/heart-rate/live endpoint in real time.

### Fall Detection

MPU6050 measures acceleration along X, Y, Z axes.

Calculates “jerk” (rate of acceleration change).

If jerk exceeds fallThreshold, a fall is detected.

Buzzer sounds; the user has 5 seconds to cancel alarm by pressing the button.

If no response, GPS coordinates are acquired and sent to /api/fall-detection.

### Button Functionality

Single press: stops buzzer alarm.

Double press: reserved for future SOS or manual alerts.

Wi-Fi & Server Communication

Auto-connects to configured Wi-Fi network.

Sends data using HTTP POST requests with JSON payload and JWT authentication.

---

# 🚀 Quick Start

**1.Clone the repository:**  
     
     git clone https://github.com/Brammagirisk/askabhi.git
     cd askabhi

Configure Wi-Fi and API settings in the source code
Build and upload the firmware to ESP32 using Arduino IDE or PlatformIO
See documentation for detailed hardware wiring and server API setup

**2.Password credentials :**  
     
     #define WIFI_SSID     "Name of your hotspot"
     #define WIFI_PASSWORD "Password"
     
---

 # 📦 Hardware Components


**ESP32 Development BoardI**

**Main microcontroller and Wi-Fi**

**MAX30102 Sensor**

**Heart rate optical sensor**

**MPU6050**

**Accelerometer for fall detection**

**Neo-6M GPS Module**

**Location tracking**

**Buzzer**

**Emergency alert**

**Push Button**

**Alarm cancel control**

---

# 📡 Server API Endpoints

Heart Rate Live Update: /api/heart-rate/live
Fall Detection Alert: /api/fall-detection
Both require JWT authentication token in HTTP headers.
