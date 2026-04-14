# 🤖 Robo-Eyes: Expressive OLED Emotion Engine

An interactive animation system for robots that brings character to life using an ESP32 and an SSD1306 OLED display. This project features a real-time web dashboard to toggle between different emotional states.

## 🛠️ Tech Stack
![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)
![ESP32](https://img.shields.io/badge/ESP32-S3-E7352C?style=for-the-badge&logo=espressif&logoColor=white)
![OLED](https://img.shields.io/badge/Display-SSD1306_I2C-blue?style=for-the-badge)

---

## 📸 Animation Gallery

| 3d Printed Car Chassis | UI Control Panel |
| :---: | :---: |
| ![Car Chassis](./images/eyes_angry.png) | ![Control Panel](./images/eyes_web_ui.png) |
| *3d printed mini car with custom soldered pcb* | *Dark-themed mobile control grid* |

---

## ✨ Features
* **Procedural Eye Darting:** In Neutral and Happy modes, the eyes "look around" randomly to simulate curiosity and life.
* **Complex Masking:** Uses geometric primitives (triangles and roundrects) to create realistic expressions like "Angry," "Sad," and "Happy" without high-memory bitmaps.
* **Emotion Persistence:** Once an emotion is selected via the web interface, it remains active indefinitely until the next command is received.
* **Specialized Animations:** * **Crying:** Animated teardrop particles moving down the Y-axis.
    * **Winking:** Asymmetrical blinking logic where only the right eye closes.
    * **Love:** Heart-shaped eyes drawn using geometric circle/triangle composition.

## ⚙️ Logic Breakdown
The animation engine uses a **Non-Blocking Timer Pattern** to handle multiple events simultaneously:
1. **Blink Timer:** Triggers physiological blinks at varied intervals based on current mood (e.g., surprised eyes blink faster).
2. **Idle Timer:** Controls the `lookOffsetX/Y` variables for pupil movement.
3. **Particle Timer:** Manages the `tearDropY` position for the crying effect.

## 🌐 Web Interface
The ESP32 acts as a **Soft Access Point** (`RoboEyes_ESP32`). Simply connect and navigate to the IP address to access the touch-optimized control grid.

---
**Developed by:** Swarnendu Kundu  
*Advancing Robotics through Human-Centered Design*
