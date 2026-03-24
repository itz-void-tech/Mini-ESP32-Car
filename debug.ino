#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include <Adafruit_VL53L0X.h> // Required for ST's proprietary ToF init sequence

// ==========================================
// PINS & HARDWARE CONFIGURATION
// ==========================================
#define I2C_SDA 21
#define I2C_SCL 22

// Motor Driver Pins (TB6612FNG)
#define PWMA 25
#define AIN1 26
#define AIN2 27
#define PWMB 14
#define BIN1 12  // CAUTION: Strapping pin. Do not pull HIGH externally at boot.
#define BIN2 13
#define STBY 33

// I2C Addresses
#define MPU_ADDR 0x68
#define QMC_ADDR 0x0D
#define HMC_ADDR 0x1E
#define VL53_ADDR 0x29

// Tuning & Constants
const int REQUIRED_UNIQUE_SAMPLES = 800;
const float MPU_ALPHA = 0.2; // Low-pass filter coefficient
float declination = 0.0;     // Adjust for local geographic declination

// ==========================================
// GLOBALS & STATE
// ==========================================
WebServer server(80);
Preferences preferences;
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

// Hardware Status Flags
bool mpuOk = false;
bool magOk = false;
bool vl53Ok = false;
uint8_t magAddr = 0x00;
String magType = "NONE";
bool isHMC = false;

// Sensor Data
float accX = 0, accY = 0, accZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float magX = 0, magY = 0, magZ = 0;
float heading = 0.0;
float magOffsetX = 0, magOffsetY = 0, magOffsetZ = 0;
uint16_t distanceMM = 0;

// Calibration State Machine
bool isCalibratingMag = false;
int magCalSamplesCollected = 0;
int16_t lastRawX = 0, lastRawY = 0, lastRawZ = 0; 
float magMinX = 32767, magMaxX = -32768;
float magMinY = 32767, magMaxY = -32768;
float magMinZ = 32767, magMaxZ = -32768;

// Timing & Non-Blocking Logic
unsigned long lastConsoleUpdate = 0;
unsigned long lastMotorChange = 0;
unsigned long lastI2CRetry = 0;
int motorDiagnosticState = 0; 
String motorLogicStatus = "INIT";

// ==========================================
// WEB DASHBOARD (PROGMEM)
// ==========================================
const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Advanced Telemetry</title>
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
    table { width: 100%; text-align: left; font-size: 0.85em; }
    th, td { padding: 5px; border-bottom: 1px solid #30363d; }
    th { color: #8b949e; width: 45%; }
    
    #progress-container { width: 100%; background-color: #30363d; border-radius: 5px; margin: 10px 0; display: none; }
    #progress-bar { width: 0%; height: 20px; background-color: #238636; text-align: center; color: white; border-radius: 5px; transition: width 0.1s; }
    
    #compass-svg { width: 150px; height: 150px; display: block; margin: 10px auto; }
    .compass-ring { stroke: #30363d; stroke-width: 2; fill: none; }
    .compass-degree-mark { stroke: #30363d; stroke-width: 1; }
    .compass-text { fill: #8b949e; font-size: 14px; text-anchor: middle;}
    .compass-text-main { fill: #ffffff; font-weight: bold; font-size: 18px;}
    #compass-needle { fill: #da3633; transition: transform 0.2s ease-out; transform-origin: 75px 75px; }
    #heading-val { font-size: 2em; color: #58a6ff; text-align: center; margin: 0; }
    #dist-val { font-size: 1.5em; color: #f2cc60; text-align: center; margin-top: 5px; }
    
    .ok { color: #00ff00; font-weight: bold; }
    .err { color: #da3633; font-weight: bold; }
    .warn { color: #f2cc60; font-weight: bold; }
  </style>
</head>
<body>
  <h1>[ ESP32 ] SPY CAR DIAGNOSTICS</h1>
  
  <div class="container">
    <div class="card">
      <div id="heading-val">0.0&deg;</div>
      <div id="dist-val">RANGE: -- mm</div>
      <svg id="compass-svg" viewBox="0 0 150 150">
        <circle class="compass-ring" cx="75" cy="75" r="70"/>
        <line class="compass-degree-mark" x1="75" y1="5" x2="75" y2="15" /> <line class="compass-degree-mark" x1="145" y1="75" x2="135" y2="75" /> <line class="compass-degree-mark" x1="75" y1="145" x2="75" y2="135" /> <line class="compass-degree-mark" x1="5" y1="75" x2="15" y2="75" /> 
        <text x="75" y="30" class="compass-text compass-text-main">N</text><text x="130" y="80" class="compass-text">E</text><text x="75" y="135" class="compass-text">S</text><text x="20" y="80" class="compass-text">W</text>
        <g id="compass-needle"><polygon points="75,10 85,75 75,90 65,75" /><circle cx="75" cy="75" r="5" fill="#58a6ff"/></g>
      </svg>
    </div>

    <div class="card">
      <h3>SYSTEM DATA</h3>
      <div id="data">Loading telemetry...</div>
    </div>

    <div class="card">
      <h3>MAG CALIBRATION</h3>
      <div id="progress-container"><div id="progress-bar">0%</div></div>
      <p id="cal-status" class="ok" style="text-align:center;">IDLE (Offsets Active)</p>
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
          document.getElementById('dist-val').innerHTML = `RANGE: ${data.dist} mm`;

          if(data.calibrating) {
            document.getElementById('cal-status').innerText = `COLLECTING DATA...`;
            document.getElementById('cal-status').className = "warn";
            document.getElementById('progress-bar').style.width = `${data.cal_pct}%`;
            document.getElementById('progress-bar').innerText = `${data.cal_pct}%`;
            document.getElementById('progress-container').style.display = 'block';
          } else {
            document.getElementById('cal-status').innerText = "IDLE (Offsets Active)";
            document.getElementById('cal-status').className = "ok";
            document.getElementById('progress-container').style.display = 'none';
          }

          document.getElementById('data').innerHTML = `
            <table>
              <tr><th>MPU6050 Status</th><td class="${data.mpuOk ? 'ok' : 'err'}">${data.mpuOk ? 'ONLINE' : 'OFFLINE'}</td></tr>
              <tr><th>Magnetometer Status</th><td class="${data.magOk ? 'ok' : 'err'}">${data.magOk ? 'ONLINE ('+data.magType+')' : 'OFFLINE'}</td></tr>
              <tr><th>VL53L0X Status</th><td class="${data.vl53Ok ? 'ok' : 'err'}">${data.vl53Ok ? 'ONLINE' : 'OFFLINE'}</td></tr>
              <tr><th>Motor Logic</th><td style="color:#58a6ff; font-weight:bold;">${data.motorLog}</td></tr>
              <tr><th>Accel (X,Y,Z)</th><td>${data.aX.toFixed(2)}, ${data.aY.toFixed(2)}, ${data.aZ.toFixed(2)} g</td></tr>
              <tr><th>Gyro (X,Y,Z)</th><td>${data.gX.toFixed(1)}, ${data.gY.toFixed(1)}, ${data.gZ.toFixed(1)} &deg;/s</td></tr>
            </table>
          `;
        });
    }, 200); 
  </script>
</body>
</html>
)=====";

// ==========================================
// MOTOR DRIVER LOGIC DIAGNOSTICS
// ==========================================
void initMotorDriver() {
  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  
  digitalWrite(STBY, LOW); // Start disabled
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); digitalWrite(PWMA, LOW);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); digitalWrite(PWMB, LOW);
  Serial.println("[MOTOR] Pins configured. Driver currently DISABLED (STBY LOW).");
}

void testMotorDriver() {
  // Non-blocking logic diagnostic sequence (Cycles every 3 seconds)
  if (millis() - lastMotorChange > 3000) {
    lastMotorChange = millis();
    motorDiagnosticState = (motorDiagnosticState + 1) % 4;
    
    // Ensure driver is enabled for tests
    digitalWrite(STBY, HIGH);
    
    switch(motorDiagnosticState) {
      case 0: // STOP
        digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); digitalWrite(PWMA, LOW);
        digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); digitalWrite(PWMB, LOW);
        motorLogicStatus = "STOP SIGNAL";
        Serial.println("\n[MOTOR] STBY HIGH -> DRIVER ENABLED");
        Serial.println("[MOTOR] STOP SIGNAL SENT");
        break;
        
      case 1: // FORWARD
        digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW); digitalWrite(PWMA, HIGH);
        digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); digitalWrite(PWMB, HIGH);
        motorLogicStatus = "FORWARD SIGNAL";
        Serial.println("\n[MOTOR] AIN1 HIGH / AIN2 LOW -> FORWARD SIGNAL");
        Serial.println("[MOTOR] BIN1 HIGH / BIN2 LOW -> FORWARD SIGNAL");
        break;
        
      case 2: // STOP
        digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); digitalWrite(PWMA, LOW);
        digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); digitalWrite(PWMB, LOW);
        motorLogicStatus = "STOP SIGNAL";
        Serial.println("\n[MOTOR] STOP SIGNAL SENT");
        break;
        
      case 3: // REVERSE
        digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH); digitalWrite(PWMA, HIGH);
        digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); digitalWrite(PWMB, HIGH);
        motorLogicStatus = "REVERSE SIGNAL";
        Serial.println("\n[MOTOR] AIN1 LOW / AIN2 HIGH -> REVERSE SIGNAL");
        Serial.println("[MOTOR] BIN1 LOW / BIN2 HIGH -> REVERSE SIGNAL");
        break;
    }
  }
}

// ==========================================
// I2C MULTI-SENSOR SETUP & RETRY LOGIC
// ==========================================
void scanI2C() {
  Serial.println("\n[I2C] Scanning bus...");
  for(byte address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[I2C] Found device at 0x%02X\n", address);
    }
  }
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Wake register
  Wire.write(0x00);
  if (Wire.endTransmission() == 0) {
    mpuOk = true;
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(); // 500deg/s
    Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x08); Wire.endTransmission(); // 4g
    Serial.println("[MPU6050] ONLINE & CONFIGURED");
    
    // Quick startup calibration
    long sumX = 0, sumY = 0, sumZ = 0;
    for (int i = 0; i < 500; i++) {
      Wire.beginTransmission(MPU_ADDR); Wire.write(0x43);
      if(Wire.endTransmission(false) == 0 && Wire.requestFrom(MPU_ADDR, 6, true) == 6) {
        sumX += (int16_t)(Wire.read() << 8 | Wire.read());
        sumY += (int16_t)(Wire.read() << 8 | Wire.read());
        sumZ += (int16_t)(Wire.read() << 8 | Wire.read());
      }
      delay(2);
    }
    gyroBiasX = (float)sumX / 500.0; gyroBiasY = (float)sumY / 500.0; gyroBiasZ = (float)sumZ / 500.0;
  } else {
    mpuOk = false;
    Serial.println("[MPU6050] ERROR: NOT DETECTED");
  }
}

void initMag() {
  Wire.beginTransmission(QMC_ADDR);
  if (Wire.endTransmission() == 0) {
    magOk = true; magAddr = QMC_ADDR; magType = "QMC5883L"; isHMC = false;
    Wire.beginTransmission(magAddr); Wire.write(0x09); Wire.write(0x1D); Wire.endTransmission();
    Wire.beginTransmission(magAddr); Wire.write(0x0B); Wire.write(0x01); Wire.endTransmission();
    Serial.println("[MAG] ONLINE (QMC5883L)");
    return;
  }
  
  Wire.beginTransmission(HMC_ADDR);
  if (Wire.endTransmission() == 0) {
    magOk = true; magAddr = HMC_ADDR; magType = "HMC5883L"; isHMC = true;
    Wire.beginTransmission(magAddr); Wire.write(0x00); Wire.write(0x70); Wire.endTransmission();
    Wire.beginTransmission(magAddr); Wire.write(0x01); Wire.write(0xA0); Wire.endTransmission();
    Wire.beginTransmission(magAddr); Wire.write(0x02); Wire.write(0x00); Wire.endTransmission();
    Serial.println("[MAG] ONLINE (HMC5883L)");
    return;
  }
  magOk = false;
  Serial.println("[MAG] ERROR: NOT DETECTED");
}

void initVL53() {
  Wire.beginTransmission(VL53_ADDR);
  if (Wire.endTransmission() == 0) {
    if (lox.begin()) {
      vl53Ok = true;
      Serial.println("[VL53L0X] ONLINE & CONFIGURED");
      return;
    }
  }
  vl53Ok = false;
  Serial.println("[VL53L0X] ERROR: NOT DETECTED");
}

// ==========================================
// SENSOR READING (NON-BLOCKING)
// ==========================================
void readMPU() {
  if (!mpuOk) return;
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(MPU_ADDR, 14, true) != 14) { 
    mpuOk = false; return; 
  }
  
  float rawAX = (int16_t)(Wire.read() << 8 | Wire.read()) / 8192.0;
  float rawAY = (int16_t)(Wire.read() << 8 | Wire.read()) / 8192.0;
  float rawAZ = (int16_t)(Wire.read() << 8 | Wire.read()) / 8192.0;
  Wire.read(); Wire.read(); // Skip Temp
  float rawGX = ((int16_t)(Wire.read() << 8 | Wire.read()) - gyroBiasX) / 65.5;
  float rawGY = ((int16_t)(Wire.read() << 8 | Wire.read()) - gyroBiasY) / 65.5;
  float rawGZ = ((int16_t)(Wire.read() << 8 | Wire.read()) - gyroBiasZ) / 65.5;

  // EMA Filter
  accX = (MPU_ALPHA * rawAX) + ((1.0 - MPU_ALPHA) * accX);
  accY = (MPU_ALPHA * rawAY) + ((1.0 - MPU_ALPHA) * accY);
  accZ = (MPU_ALPHA * rawAZ) + ((1.0 - MPU_ALPHA) * accZ);
  gyroX = (MPU_ALPHA * rawGX) + ((1.0 - MPU_ALPHA) * gyroX);
  gyroY = (MPU_ALPHA * rawGY) + ((1.0 - MPU_ALPHA) * gyroY);
  gyroZ = (MPU_ALPHA * rawGZ) + ((1.0 - MPU_ALPHA) * gyroZ);
}

void readMag() {
  if (!magOk) return;
  int16_t rawX=0, rawY=0, rawZ=0;

  Wire.beginTransmission(magAddr);
  Wire.write(isHMC ? 0x03 : 0x00); 
  if(Wire.endTransmission(false) != 0 || Wire.requestFrom((uint8_t)magAddr, (uint8_t)6, (uint8_t)true) != 6) { 
    magOk = false; return; 
  }

  if (isHMC) {
    rawX = (Wire.read() << 8) | Wire.read(); rawZ = (Wire.read() << 8) | Wire.read(); rawY = (Wire.read() << 8) | Wire.read();
  } else {
    rawX = Wire.read() | (Wire.read() << 8); rawY = Wire.read() | (Wire.read() << 8); rawZ = Wire.read() | (Wire.read() << 8);
  }

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
      magOffsetX = (magMaxX + magMinX) / 2.0; magOffsetY = (magMaxY + magMinY) / 2.0; magOffsetZ = (magMaxZ + magMinZ) / 2.0;
      preferences.begin("mag", false); preferences.putFloat("x", magOffsetX); preferences.putFloat("y", magOffsetY); preferences.end();
      Serial.println("\n[MAG] Calibration Saved.");
    }
  }
  magX = rawX - magOffsetX; magY = rawY - magOffsetY; magZ = rawZ - magOffsetZ;
}

void readVL53() {
  if (!vl53Ok) return;
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false); 
  if (measure.RangeStatus != 4) {  // 4 = Out of range
    distanceMM = measure.RangeMilliMeter;
  } else {
    distanceMM = 8190; // Standard out-of-range flag value
  }
}

void calculateHeading() {
  if (!magOk || (magX == 0 && magY == 0)) return;
  heading = atan2(magY, magX) * 180.0 / M_PI;
  heading += declination;
  if (heading < 0) heading += 360.0;
  if (heading >= 360.0) heading -= 360.0;
}

// ==========================================
// WEB SERVER SETUP
// ==========================================
void setupWebServer() {
  server.on("/", []() { server.send(200, "text/html", INDEX_HTML); });
  server.on("/data", []() {
    int progress = isCalibratingMag ? (int)(((float)magCalSamplesCollected / REQUIRED_UNIQUE_SAMPLES) * 100) : 0;
    if (progress > 100) progress = 100;

    String json = "{";
    json += "\"mpuOk\":" + String(mpuOk ? "true" : "false") + ",\"magOk\":" + String(magOk ? "true" : "false") + ",\"vl53Ok\":" + String(vl53Ok ? "true" : "false") + ",";
    json += "\"magType\":\"" + magType + "\",\"motorLog\":\"" + motorLogicStatus + "\",\"heading\":" + String(heading, 1) + ",\"dist\":" + String(distanceMM) + ",";
    json += "\"aX\":" + String(accX, 2) + ",\"aY\":" + String(accY, 2) + ",\"aZ\":" + String(accZ, 2) + ",";
    json += "\"gX\":" + String(gyroX, 1) + ",\"gY\":" + String(gyroY, 1) + ",\"gZ\":" + String(gyroZ, 1) + ",";
    json += "\"calibrating\":" + String(isCalibratingMag ? "true" : "false") + ",\"cal_pct\":" + String(progress) + "}";
    server.send(200, "application/json", json);
  });
  server.on("/start_cal", []() {
    magMinX = 32767; magMaxX = -32768; magMinY = 32767; magMaxY = -32768; magMinZ = 32767; magMaxZ = -32768;
    magCalSamplesCollected = 0; lastRawX = 0; lastRawY = 0; lastRawZ = 0;
    isCalibratingMag = true;
    server.send(200, "text/plain", "OK");
  });
  server.on("/reset_cal", []() {
    magOffsetX = 0; magOffsetY = 0; magOffsetZ = 0;
    preferences.begin("mag", false); preferences.putFloat("x", 0); preferences.putFloat("y", 0); preferences.end();
    server.send(200, "text/plain", "OK");
  });
  server.begin();
}

// ==========================================
// MAIN LOOP & SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(1000); 
  Serial.println("\n\n================================================");
  Serial.println("[ SYSTEM ] ESP32 SPY CAR DIAGNOSTICS INIT...");
  
  initMotorDriver();
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); 
  
  scanI2C();
  initMPU();
  initMag();
  initVL53();
  
  preferences.begin("mag", true); 
  magOffsetX = preferences.getFloat("x", 0.0); magOffsetY = preferences.getFloat("y", 0.0); 
  preferences.end();
  
  WiFi.mode(WIFI_AP); 
  WiFi.softAP("SPY_CAR_DIAG", "12345678");
  Serial.print("[WIFI] AP Mode Active. IP: "); Serial.println(WiFi.softAPIP());
  
  setupWebServer();
  Serial.println("================================================\n");
}

void loop() {
  readMPU();
  readMag(); 
  readVL53();
  calculateHeading();
  
  testMotorDriver(); 
  server.handleClient();
  
  // Console Logging & Sensor Self-Healing
  if (millis() - lastConsoleUpdate > 1000) {
    if (!isCalibratingMag) {
      Serial.printf("[TELEM] Head: %05.1f | Dist: %04dmm | MPU: %d | MAG: %d | VL53: %d\n", 
                    heading, distanceMM, mpuOk, magOk, vl53Ok);
    }
    lastConsoleUpdate = millis();
  }

  // Periodic recovery attempt for failed I2C devices
  if (millis() - lastI2CRetry > 5000) {
    if (!mpuOk) { Serial.println("[RECOVERY] Attempting MPU reset..."); initMPU(); }
    if (!magOk) { Serial.println("[RECOVERY] Attempting MAG reset..."); initMag(); }
    if (!vl53Ok) { Serial.println("[RECOVERY] Attempting VL53 reset..."); initVL53(); }
    lastI2CRetry = millis();
  }
}
