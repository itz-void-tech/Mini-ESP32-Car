#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>

// ==========================================
// PINS & CONFIGURATION
// ==========================================
// I2C Pins
#define I2C_SDA 21
#define I2C_SCL 22

// Motor Driver Pins (TB6612FNG)
#define PWMA 25
#define AIN1 26
#define AIN2 27
#define PWMB 14
#define BIN1 12
#define BIN2 13
#define STBY 33

// I2C Addresses
#define MPU_ADDR 0x68
#define QMC_ADDR 0x0D
#define HMC_ADDR 0x1E

// Calibration & Filtering Tuning
const int REQUIRED_UNIQUE_SAMPLES = 800;
const float MPU_ALPHA = 0.2; // Low-pass filter coefficient for MPU (0.0 - 1.0)
float declination = 0.0;     // Adjust based on your geographic location

// ==========================================
// GLOBALS & STATE
// ==========================================
WebServer server(80);
Preferences preferences;

// Sensor Status Flags
bool mpuOk = false;
bool magOk = false;
uint8_t magAddr = 0x00;
String magType = "NONE";
bool isHMC = false;

// Filtered MPU Sensor Data
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;

// Magnetometer Data
float magX = 0, magY = 0, magZ = 0;
float heading = 0.0;
float magOffsetX = 0, magOffsetY = 0, magOffsetZ = 0;

// Dynamic Mag Calibration State Machine
bool isCalibratingMag = false;
int magCalSamplesCollected = 0;
int16_t lastRawX = 0, lastRawY = 0, lastRawZ = 0; 
float magMinX = 32767, magMaxX = -32768;
float magMinY = 32767, magMaxY = -32768;
float magMinZ = 32767, magMaxZ = -32768;

// Timing & Non-blocking State Machines
unsigned long lastConsoleUpdate = 0;
unsigned long lastMotorChange = 0;
int motorState = 0; // 0=Stop, 1=Fwd, 2=Stop, 3=Rev
String motorStatusStr = "STOPPED";

// ==========================================
// WEB DASHBOARD HTML/CSS/JS (PROGMEM)
// ==========================================
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Spy Car Telemetry</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { background-color: #0d1117; color: #00ff00; font-family: 'Courier New', Courier, monospace; margin: 10px; display: flex; flex-direction: column; align-items: center;}
    h1 { color: #58a6ff; border-bottom: 1px solid #30363d; padding-bottom: 10px; width: 100%; text-align: center; font-size: 1.2em;}
    .container { max-width: 600px; width: 100%; }
    .card { background: #161b22; border: 1px solid #30363d; border-radius: 6px; padding: 15px; margin-bottom: 15px; }
    button { background: #238636; color: white; border: none; padding: 10px 20px; border-radius: 5px; cursor: pointer; font-weight: bold; margin-right: 10px; width: 100%; margin-bottom: 10px;}
    button:hover { background: #2ea043; }
    .btn-danger { background: #da3633; }
    .btn-danger:hover { background: #f85149; }
    button:disabled { background: #444; cursor: not-allowed; }
    table { width: 100%; text-align: left; font-size: 0.85em; }
    th, td { padding: 5px; border-bottom: 1px solid #30363d; }
    th { color: #8b949e; width: 40%; }
    
    #progress-container { width: 100%; background-color: #30363d; border-radius: 5px; margin: 10px 0; display: none; }
    #progress-bar { width: 0%; height: 20px; background-color: #238636; text-align: center; line-height: 20px; color: white; border-radius: 5px; transition: width 0.1s; }
    #instructions { color: #8b949e; font-size: 0.85em; display: none; border-left: 3px solid #f2cc60; padding-left: 10px; margin-bottom: 10px;}

    #compass-svg { width: 150px; height: 150px; display: block; margin: 10px auto; }
    .compass-ring { stroke: #30363d; stroke-width: 2; fill: none; }
    .compass-degree-mark { stroke: #30363d; stroke-width: 1; }
    .compass-text { fill: #8b949e; font-size: 14px; text-anchor: middle; font-family: sans-serif;}
    .compass-text-main { fill: #ffffff; font-weight: bold; font-size: 18px;}
    #compass-needle { fill: #da3633; transition: transform 0.2s ease-out; transform-origin: 75px 75px; }
    #heading-val { font-size: 2em; color: #58a6ff; text-align: center; margin: 0; }
    
    .status-ok { color: #00ff00; font-weight: bold; }
    .status-err { color: #da3633; font-weight: bold; }
    .status-warn { color: #f2cc60; font-weight: bold; }
  </style>
</head>
<body>
  <h1>[ ESP32 ] SPY CAR TELEMETRY</h1>
  
  <div class="container">
    <div class="card">
      <div id="heading-val">0.0&deg;</div>
      <svg id="compass-svg" viewBox="0 0 150 150">
        <circle class="compass-ring" cx="75" cy="75" r="70"/>
        <line class="compass-degree-mark" x1="75" y1="5" x2="75" y2="15" /> <line class="compass-degree-mark" x1="145" y1="75" x2="135" y2="75" /> <line class="compass-degree-mark" x1="75" y1="145" x2="75" y2="135" /> <line class="compass-degree-mark" x1="5" y1="75" x2="15" y2="75" /> 
        <text x="75" y="30" class="compass-text compass-text-main">N</text>
        <text x="130" y="80" class="compass-text">E</text>
        <text x="75" y="135" class="compass-text">S</text>
        <text x="20" y="80" class="compass-text">W</text>
        <g id="compass-needle">
          <polygon points="75,10 85,75 75,90 65,75" />
          <circle cx="75" cy="75" r="5" fill="#58a6ff"/>
        </g>
      </svg>
    </div>

    <div class="card">
      <h3>SYSTEM DATA</h3>
      <div id="data">Loading telemetry...</div>
    </div>

    <div class="card">
      <h3>MAG CALIBRATION</h3>
      <div id="instructions">
        <strong>ACTION REQUIRED:</strong><br>
        1. Keep flat, then rotate in a slow 3D Figure-8.<br>
        2. Keep away from motor magnets during cal.<br>
        <em>Completes automatically at 100%.</em>
      </div>
      <div id="progress-container"><div id="progress-bar">0%</div></div>
      <p id="cal-status" class="status-ok" style="text-align:center;">IDLE (Offsets Active)</p>
      <button id="btn-start" onclick="fetch('/start_cal')">Start Calibration</button>
      <button class="btn-danger" id="btn-reset" onclick="fetch('/reset_cal')">Reset Offsets</button>
    </div>
  </div>

  <script>
    setInterval(() => {
      fetch('/data')
        .then(response => response.json())
        .then(data => {
          document.getElementById('compass-needle').style.transform = `rotate(${data.heading}deg)`;
          document.getElementById('heading-val').innerHTML = `${data.heading.toFixed(1)}&deg;`;

          if(data.calibrating) {
            document.getElementById('cal-status').innerText = `COLLECTING DATA...`;
            document.getElementById('cal-status').className = "status-warn";
            document.getElementById('progress-bar').style.width = `${data.cal_pct}%`;
            document.getElementById('progress-bar').innerText = `${data.cal_pct}%`;
            document.getElementById('instructions').style.display = 'block';
            document.getElementById('progress-container').style.display = 'block';
            document.getElementById('btn-start').disabled = true;
            document.getElementById('btn-reset').disabled = true;
          } else {
            document.getElementById('cal-status').innerText = "IDLE (Offsets Active)";
            document.getElementById('cal-status').className = "status-ok";
            document.getElementById('instructions').style.display = 'none';
            document.getElementById('progress-container').style.display = 'none';
            document.getElementById('btn-start').disabled = false;
            document.getElementById('btn-reset').disabled = false;
          }

          document.getElementById('data').innerHTML = `
            <table>
              <tr><th>MPU6050 Status</th><td class="${data.mpuOk ? 'status-ok' : 'status-err'}">${data.mpuOk ? 'ONLINE' : 'OFFLINE / I2C ERROR'}</td></tr>
              <tr><th>Magnetometer Status</th><td class="${data.magOk ? 'status-ok' : 'status-err'}">${data.magOk ? 'ONLINE ('+data.magType+')' : 'OFFLINE'}</td></tr>
              <tr><th>Motor State</th><td style="color:#58a6ff; font-weight:bold;">${data.motorState}</td></tr>
              <tr><th>Accel (X,Y,Z)</th><td>${data.aX.toFixed(2)}, ${data.aY.toFixed(2)}, ${data.aZ.toFixed(2)} g</td></tr>
              <tr><th>Gyro (X,Y,Z)</th><td>${data.gX.toFixed(1)}, ${data.gY.toFixed(1)}, ${data.gZ.toFixed(1)} &deg;/s</td></tr>
              <tr><th>Mag Offsets</th><td>X:${data.offX}, Y:${data.offY}, Z:${data.offZ}</td></tr>
            </table>
          `;
        }).catch(err => console.log("Fetch Error", err));
    }, 200); 
  </script>
</body>
</html>
)=====";

// ==========================================
// MOTOR CONTROL FUNCTIONS (TB6612FNG)
// ==========================================
void initMotors() {
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  digitalWrite(STBY, HIGH); // Enable driver
  motorStop();
  Serial.println("[MOTORS] TB6612FNG Initialized. STBY = HIGH.");
}

void motorStop() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); digitalWrite(PWMA, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); digitalWrite(PWMB, LOW);
  motorStatusStr = "STOPPED";
}

void motorForward() {
  // Motor A FWD
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); digitalWrite(PWMA, HIGH);
  // Motor B FWD
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); digitalWrite(PWMB, HIGH);
  motorStatusStr = "FORWARD";
}

void motorReverse() {
  // Motor A REV
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); digitalWrite(PWMA, HIGH);
  // Motor B REV
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); digitalWrite(PWMB, HIGH);
  motorStatusStr = "REVERSE";
}

void processMotorTestSequence() {
  // Non-blocking state machine for testing motors (Forward -> Stop -> Reverse -> Stop)
  if (millis() - lastMotorChange > 3000) {
    lastMotorChange = millis();
    motorState = (motorState + 1) % 4;
    
    switch(motorState) {
      case 0: motorStop(); Serial.println("[MOTOR TEST] State: STOP"); break;
      case 1: motorForward(); Serial.println("[MOTOR TEST] State: FORWARD"); break;
      case 2: motorStop(); Serial.println("[MOTOR TEST] State: STOP"); break;
      case 3: motorReverse(); Serial.println("[MOTOR TEST] State: REVERSE"); break;
    }
  }
}

// ==========================================
// I2C SENSOR FUNCTIONS
// ==========================================
void scanI2C() {
  Serial.println("\n[I2C] Scanning bus...");
  byte error, address;
  int nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.printf("[I2C] Device found at 0x%02X\n", address);
      nDevices++;
    } else if (error == 4) {
      Serial.printf("[I2C] Unknown error at 0x%02X\n", address);
    }
  }
  if (nDevices == 0) Serial.println("[I2C] No devices found.");
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Power Management 1 register
  Wire.write(0x00); // Wake up
  if (Wire.endTransmission() == 0) {
    mpuOk = true;
    // Set Gyro Config to 500 deg/s
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
    // Set Accel Config to 4g
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission();
    Serial.println("[MPU6050] STATUS: ONLINE & CONFIGURED");
  } else {
    mpuOk = false;
    Serial.println("[MPU6050] ERROR: FAILED TO WAKE UP (Check wiring)");
  }
}

void calibrateGyro() {
  if (!mpuOk) return;
  Serial.println("[GYRO] Calibrating bias... Do not move vehicle.");
  long sumX = 0, sumY = 0, sumZ = 0;
  int numSamples = 1000;
  
  for (int i = 0; i < numSamples; i++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x43); // Start of Gyro registers
    if(Wire.endTransmission(false) != 0) continue; 
    
    Wire.requestFrom(MPU_ADDR, 6, true);
    if(Wire.available() == 6) {
      sumX += (int16_t)(Wire.read() << 8 | Wire.read());
      sumY += (int16_t)(Wire.read() << 8 | Wire.read());
      sumZ += (int16_t)(Wire.read() << 8 | Wire.read());
    }
    delay(2); // Short block is acceptable during startup only
  }
  gyroBiasX = (float)sumX / numSamples;
  gyroBiasY = (float)sumY / numSamples;
  gyroBiasZ = (float)sumZ / numSamples;
  Serial.printf("[GYRO] Biases -> X:%.1f Y:%.1f Z:%.1f\n", gyroBiasX, gyroBiasY, gyroBiasZ);
}

void initMag() {
  // Attempt QMC5883L
  Wire.beginTransmission(QMC_ADDR);
  if (Wire.endTransmission() == 0) {
    magOk = true; magAddr = QMC_ADDR; magType = "QMC5883L"; isHMC = false;
    // Control Register 1: OSR=512, RNG=8G, ODR=200Hz, Mode=Continuous
    Wire.beginTransmission(magAddr); Wire.write(0x09); Wire.write(0x1D); Wire.endTransmission();
    // Set/Reset Period
    Wire.beginTransmission(magAddr); Wire.write(0x0B); Wire.write(0x01); Wire.endTransmission();
    Serial.println("[MAG] STATUS: ONLINE (QMC5883L Detected)");
    return;
  }
  
  // Attempt HMC5883L
  Wire.beginTransmission(HMC_ADDR);
  if (Wire.endTransmission() == 0) {
    magOk = true; magAddr = HMC_ADDR; magType = "HMC5883L"; isHMC = true;
    // Config Register A: 8-average, 15 Hz default, normal measurement
    Wire.beginTransmission(magAddr); Wire.write(0x00); Wire.write(0x70); Wire.endTransmission();
    // Config Register B: Gain=1090 LSb/Gauss
    Wire.beginTransmission(magAddr); Wire.write(0x01); Wire.write(0xA0); Wire.endTransmission();
    // Mode Register: Continuous Measurement
    Wire.beginTransmission(magAddr); Wire.write(0x02); Wire.write(0x00); Wire.endTransmission();
    Serial.println("[MAG] STATUS: ONLINE (HMC5883L Detected)");
    return;
  }
  
  magOk = false;
  Serial.println("[MAG] ERROR: NOT FOUND");
}

void loadMagOffsets() {
  preferences.begin("mag_cal", true); // Read-only
  magOffsetX = preferences.getFloat("offX", 0.0);
  magOffsetY = preferences.getFloat("offY", 0.0);
  magOffsetZ = preferences.getFloat("offZ", 0.0);
  preferences.end();
  Serial.printf("[MAG] Loaded Offsets -> X:%.0f Y:%.0f Z:%.0f\n", magOffsetX, magOffsetY, magOffsetZ);
}

void saveMagOffsets() {
  preferences.begin("mag_cal", false); // R/W
  preferences.putFloat("offX", magOffsetX);
  preferences.putFloat("offY", magOffsetY);
  preferences.putFloat("offZ", magOffsetZ);
  preferences.end();
}

void readMPU() {
  if (!mpuOk) return;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // Start at ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) { mpuOk = false; return; } // Handle sudden disconnect
  
  if(Wire.requestFrom(MPU_ADDR, 14, true) == 14) {
    // Read & Scale Accel (4g scale -> divide by 8192)
    float rawAX = (int16_t)(Wire.read() << 8 | Wire.read()) / 8192.0;
    float rawAY = (int16_t)(Wire.read() << 8 | Wire.read()) / 8192.0;
    float rawAZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 8192.0;
    Wire.read(); Wire.read(); // Skip Temp
    // Read, Bias correct, & Scale Gyro (500deg/s scale -> divide by 65.5)
    float rawGX = ((int16_t)(Wire.read() << 8 | Wire.read()) - gyroBiasX) / 65.5;
    float rawGY = ((int16_t)(Wire.read() << 8 | Wire.read()) - gyroBiasY) / 65.5;
    float rawGZ = ((int16_t)(Wire.read() << 8 | Wire.read()) - gyroBiasZ) / 65.5;

    // Apply Exponential Moving Average (EMA) Filter
    accX = (MPU_ALPHA * rawAX) + ((1.0 - MPU_ALPHA) * accX);
    accY = (MPU_ALPHA * rawAY) + ((1.0 - MPU_ALPHA) * accY);
    accZ = (MPU_ALPHA * rawAZ) + ((1.0 - MPU_ALPHA) * accZ);
    gyroX = (MPU_ALPHA * rawGX) + ((1.0 - MPU_ALPHA) * gyroX);
    gyroY = (MPU_ALPHA * rawGY) + ((1.0 - MPU_ALPHA) * gyroY);
    gyroZ = (MPU_ALPHA * rawGZ) + ((1.0 - MPU_ALPHA) * gyroZ);
  } else {
    mpuOk = false; // Flag failure if bytes not received
  }
}

void readMag() {
  if (!magOk) return;
  int16_t rawX=0, rawY=0, rawZ=0;

  if (isHMC) {
    // HMC5883L returns X, Z, Y
    Wire.beginTransmission(magAddr);
    Wire.write(0x03); 
    if(Wire.endTransmission(false) != 0) { magOk = false; return; }
    if(Wire.requestFrom((uint8_t)magAddr, (uint8_t)6, (uint8_t)true) == 6) {
      rawX = (Wire.read() << 8) | Wire.read();
      rawZ = (Wire.read() << 8) | Wire.read();
      rawY = (Wire.read() << 8) | Wire.read();
    } else magOk = false;
  } else {
    // QMC5883L returns X, Y, Z (LSB first)
    Wire.beginTransmission(magAddr);
    Wire.write(0x00);
    if(Wire.endTransmission(false) != 0) { magOk = false; return; }
    if(Wire.requestFrom((uint8_t)magAddr, (uint8_t)6, (uint8_t)true) == 6) {
      rawX = Wire.read() | (Wire.read() << 8);
      rawY = Wire.read() | (Wire.read() << 8);
      rawZ = Wire.read() | (Wire.read() << 8);
    } else magOk = false;
  }

  // Dynamic Hard-Iron Calibration Logic
  if (isCalibratingMag) {
    if (abs(rawX - lastRawX) > 15 || abs(rawY - lastRawY) > 15 || abs(rawZ - lastRawZ) > 15) {
        if (rawX < magMinX) magMinX = rawX; if (rawX > magMaxX) magMaxX = rawX;
        if (rawY < magMinY) magMinY = rawY; if (rawY > magMaxY) magMaxY = rawY;
        if (rawZ < magMinZ) magMinZ = rawZ; if (rawZ > magMaxZ) magMaxZ = rawZ;
        
        magCalSamplesCollected++;
        lastRawX = rawX; lastRawY = rawY; lastRawZ = rawZ;
    }
    
    if (magCalSamplesCollected >= REQUIRED_UNIQUE_SAMPLES) {
      isCalibratingMag = false;
      magOffsetX = (magMaxX + magMinX) / 2.0;
      magOffsetY = (magMaxY + magMinY) / 2.0;
      magOffsetZ = (magMaxZ + magMinZ) / 2.0;
      saveMagOffsets();
      Serial.println("\n[MAG] Calibration COMPLETE. Offsets Saved.");
    }
  }

  // Apply Hard-Iron Offsets
  magX = rawX - magOffsetX; 
  magY = rawY - magOffsetY; 
  magZ = rawZ - magOffsetZ;
}

void calculateHeading() {
  if (!magOk || (magX == 0 && magY == 0)) return; // Prevent atan2(0,0)
  
  heading = atan2(magY, magX) * 180.0 / M_PI;
  heading += declination;
  
  // Normalize to 0-360
  if (heading < 0) heading += 360.0;
  if (heading >= 360.0) heading -= 360.0;
}

// ==========================================
// WEB SERVER SETUP
// ==========================================
void setupWebServer() {
  server.on("/", []() { 
    server.send(200, "text/html", INDEX_HTML); 
  });

  server.on("/data", []() {
    int progress = 0;
    if (isCalibratingMag) {
        progress = (int)(((float)magCalSamplesCollected / REQUIRED_UNIQUE_SAMPLES) * 100);
        if (progress > 100) progress = 100;
    }

    String json = "{";
    json += "\"mpuOk\":" + String(mpuOk ? "true" : "false") + ",";
    json += "\"magOk\":" + String(magOk ? "true" : "false") + ",";
    json += "\"magType\":\"" + magType + "\",";
    json += "\"motorState\":\"" + motorStatusStr + "\",";
    json += "\"heading\":" + String(heading, 1) + ",";
    json += "\"aX\":" + String(accX, 2) + ",\"aY\":" + String(accY, 2) + ",\"aZ\":" + String(accZ, 2) + ",";
    json += "\"gX\":" + String(gyroX, 1) + ",\"gY\":" + String(gyroY, 1) + ",\"gZ\":" + String(gyroZ, 1) + ",";
    json += "\"offX\":" + String(magOffsetX, 0) + ",\"offY\":" + String(magOffsetY, 0) + ",\"offZ\":" + String(magOffsetZ, 0) + ",";
    json += "\"calibrating\":" + String(isCalibratingMag ? "true" : "false") + ",";
    json += "\"cal_pct\":" + String(progress);
    json += "}";
    server.send(200, "application/json", json);
  });

  server.on("/start_cal", []() {
    magMinX = 32767; magMaxX = -32768; magMinY = 32767; magMaxY = -32768; magMinZ = 32767; magMaxZ = -32768;
    magCalSamplesCollected = 0;
    lastRawX = 0; lastRawY = 0; lastRawZ = 0;
    isCalibratingMag = true;
    Serial.println("[MAG] Starting Dynamic Hard-Iron Calibration...");
    server.send(200, "text/plain", "OK");
  });

  server.on("/reset_cal", []() {
    magOffsetX = 0; magOffsetY = 0; magOffsetZ = 0;
    saveMagOffsets();
    Serial.println("[MAG] Offsets Reset to Defaults (0).");
    server.send(200, "text/plain", "OK");
  });

  server.begin();
  Serial.println("[WEB] Server started on port 80");
}

// ==========================================
// MAIN SETUP & LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("\n\n================================================");
  Serial.println("[ SYSTEM ] ESP32 SPY CAR INIT...");
  
  // Init Hardware
  initMotors();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz Fast I2C
  
  // Sensor Setup
  scanI2C();
  initMPU();
  initMag();
  loadMagOffsets();
  calibrateGyro(); 
  
  // WiFi Access Point
  WiFi.mode(WIFI_AP); 
  WiFi.softAP("SPY_CAR_SECURE", "12345678");
  Serial.print("[WIFI] AP Mode Active. Connect and go to IP: "); 
  Serial.println(WiFi.softAPIP());
  
  setupWebServer();
  Serial.println("================================================\n");
}

void loop() {
  // Non-blocking telemetry acquisition
  readMPU();
  readMag(); 
  calculateHeading();
  
  // Non-blocking Motor Test Sequence
  // REMOVE OR COMMENT OUT THIS LINE once you want to manually control motors
  processMotorTestSequence(); 
  
  // Handle Web Client Requests
  server.handleClient();
  
  // Non-blocking Serial Console Debug (1Hz)
  if (millis() - lastConsoleUpdate > 1000) {
    if (!isCalibratingMag) {
      Serial.printf("[TELEMETRY] Head: %05.1f | MPU: %s | MAG: %s | Motor: %s\n", 
                    heading, mpuOk?"OK":"FAIL", magOk?"OK":"FAIL", motorStatusStr.c_str());
    }
    // Attempt re-init if a sensor suddenly failed
    if (!mpuOk && millis() > 5000) initMPU(); 
    if (!magOk && millis() > 5000) initMag(); 
    
    lastConsoleUpdate = millis();
  }
}
