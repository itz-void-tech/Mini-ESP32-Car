# 🏎️ ESP32-S3 Spy Car: Integrated Autonomous System (V3.0)

A high-intelligence robotics platform that merges **Inertial Telemetry**, **ToF Laser Scanning**, and **Human-Robot Interaction (HRI)**. This system transforms an ESP32-S3 into a mobile command unit capable of autonomous safety and emotional expression.

## 🛠️ Tech Stack
![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![ESP32](https://img.shields.io/badge/ESP32-S3-E7352C?style=for-the-badge&logo=espressif&logoColor=white)
![JavaScript](https://img.shields.io/badge/JavaScript-F7DF1E?style=for-the-badge&logo=javascript&logoColor=black)
![I2C](https://img.shields.io/badge/Protocol-I2C-F7DF1E?style=for-the-badge)
![Power](https://img.shields.io/badge/Battery-2S_LiPo_BMS-green?style=for-the-badge)

---

## 📸 System Overview

| Full Model Assembly | Integrated Web Dashboard |
| :---: | :---: |
| ![Spy Car Full Assembly](./images/car_final.jpg) | ![Advanced Telemetry Dashboard](./images/dashboard_final.png) |
| *N20-powered chassis with ToF Laser & OLED Eyes* | *Real-time GPS Map, Compass, & Emotion Control* |

---

## 🚀 Key Features

### 📡 1. Advanced Telemetry & Navigation
* **GPS Real-Time Tracking:** Live coordinate mapping and speed calculation (km/h) via NEO-6M.
* **Dual-Sensor Compass:** Reactive SVG HUD based on QMC5883L/HMC5883L magnetic heading.
* **IMU Data Streaming:** MPU6050 6-axis telemetry (Accel/Gyro) with EMA filtering for stability.

### ⚡ 2. LiDAR & Autonomous Safety
* **ToF Laser Scanning:** VL53L0X Time-of-Flight sensor for millimetric obstacle detection.
* **Self-Healing Logic:** Automated I2C bus recovery for mission-critical reliability.
* **Safety Protocols:** Automatic crash detection, tilt/fall protection, and obstacle repulsion.

### 🎭 3. OLED Emotion Engine (HRI)
* **9 Emotional States:** Neutral, Happy, Sad, Angry, Sleepy, Surprised, Crying, Wink, and Love.
* **Procedural Animation:** Randomized eye-darting and physiological blinking for a "lifelike" feel.
* **Persistent State:** Emotions hold indefinitely until changed via the remote command center.

### 🔋 4. Power & Drive
* **High-Efficiency Drive:** TB6612FNG driver controlling high-torque N20 Micro Gear Motors.
* **Custom Power Cell:** Powered by a self-made **2S LiPo battery** with a 7.4V BMS.

---

## ⚙️ Hardware Components
* **Core:** ESP32-S3 (Dual-Core, WiFi/BT)
* **Displays:** 0.96" SSD1306 OLED (I2C)
* **Sensors:** VL53L0X (Laser), MPU6050 (IMU), HMC5883L (Mag), NEO-6M (GPS)
* **Mechanical:** 2x N20 Motors, MG90S Pan/Tilt Servos
* **Power:** Custom 2S LiPo Pack with BMS

## 🌐 Mission Control
Connect to the WiFi AP `SPY_CAR_DIAG` to access the unified dashboard. This interface allows for simultaneous motor control, camera pan/tilt, GPS observation, and emotional state selection.

---
**Developed by:** Swarnendu Kundu  
*Robotics, Embedded Systems, & Full-Stack IoT*
