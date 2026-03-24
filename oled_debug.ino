#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// --- WIFI & WEB SERVER CONFIG ---
const char* ssid = "RoboEyes_ESP32"; 
const char* password = "password123"; 
WebServer server(80);

// --- EMOTION STATES ---
enum Emotion { NEUTRAL, HAPPY, SAD, ANGRY, SLEEPY, SURPRISED, CRYING, WINK, LOVE };
Emotion currentEmotion = NEUTRAL;

// --- TIMING & ANIMATION VARIABLES ---
unsigned long previousMillis = 0;
unsigned long blinkTimer = 0;
unsigned long idleTimer = 0;

bool isBlinking = false;
int lookOffsetX = 0;
int lookOffsetY = 0;
int tearDropY = 0;

// Configurable timing
const int blinkIntervalNormal = 3000; 
const int blinkDuration = 150;        
const int idleInterval = 2000;        

// ==========================================
// HTML & JS (Web Interface) - EMOJIS REMOVED
// ==========================================
const char* htmlPage PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Robo-Eye Control</title>
  <style>
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background: #121212; color: white; text-align: center; margin: 0; padding: 20px; }
    h2 { color: #00ffcc; margin-bottom: 30px; letter-spacing: 1px;}
    .grid { display: grid; grid-template-columns: repeat(3, 1fr); gap: 15px; max-width: 450px; margin: auto; }
    button { background: #222; border: 2px solid #00ffcc; color: white; padding: 20px 10px; font-size: 16px; font-weight: bold; border-radius: 8px; cursor: pointer; transition: 0.2s; text-transform: uppercase;}
    button:active { background: #00ffcc; color: black; transform: scale(0.95); }
  </style>
  <script>
    function setEmotion(emo) {
      fetch('/set?emo=' + emo).then(response => console.log('Emotion changed: ' + emo));
    }
  </script>
</head>
<body>
  <h2>ROBO-EYE CONTROL</h2>
  <div class="grid">
    <button onclick="setEmotion('neutral')">Neutral</button>
    <button onclick="setEmotion('happy')">Happy</button>
    <button onclick="setEmotion('sad')">Sad</button>
    <button onclick="setEmotion('angry')">Angry</button>
    <button onclick="setEmotion('sleepy')">Sleepy</button>
    <button onclick="setEmotion('surprised')">Surprised</button>
    <button onclick="setEmotion('crying')">Crying</button>
    <button onclick="setEmotion('wink')">Wink</button>
    <button onclick="setEmotion('love')">Love</button>
  </div>
</body>
</html>
)rawliteral";

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(10, 25);
  display.print("Starting AP...");
  display.display();

  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() {
    server.send(200, "text/html", htmlPage);
  });

  server.on("/set", handleEmotionRequest);
  server.begin();
  
  delay(1000); 
}

// ==========================================
// MAIN LOOP
// ==========================================
void loop() {
  server.handleClient(); 
  updateLogic();         
  drawScreen();          
}

// ==========================================
// LOGIC & TIMERS
// ==========================================
void updateLogic() {
  unsigned long currentMillis = millis();

  // 1. Blinking Logic
  if (!isBlinking) {
    int currentBlinkInterval = (currentEmotion == SLEEPY) ? 4000 : 
                               (currentEmotion == SURPRISED) ? 1500 : 
                               (currentEmotion == WINK) ? 2000 : blinkIntervalNormal;
                               
    if (currentMillis - blinkTimer >= currentBlinkInterval + random(-500, 1000)) {
      isBlinking = true;
      blinkTimer = currentMillis;
    }
  } else {
    int currentBlinkDuration = (currentEmotion == SLEEPY) ? 300 : blinkDuration;
    if (currentMillis - blinkTimer >= currentBlinkDuration) {
      isBlinking = false;
      blinkTimer = currentMillis;
    }
  }

  // 2. Idle Eye Darting (Only look around if neutral or happy)
  if (currentMillis - idleTimer >= idleInterval + random(0, 2000)) {
    idleTimer = currentMillis;
    if (currentEmotion == NEUTRAL || currentEmotion == HAPPY) {
      lookOffsetX = random(-6, 7); 
      lookOffsetY = random(-4, 5); 
    } else {
      lookOffsetX = 0; lookOffsetY = 0;
    }
  }

  // 3. Crying Animation Logic
  if (currentEmotion == CRYING) {
    tearDropY += 3;
    if (tearDropY > 64) tearDropY = 32; 
  } else {
    tearDropY = 32;
  }
  
  // NOTE: Auto-reset logic was completely removed here. 
  // Emotions will now hold indefinitely.
}

// ==========================================
// WEB REQUEST HANDLER
// ==========================================
void handleEmotionRequest() {
  if (server.hasArg("emo")) {
    String emo = server.arg("emo");
    Serial.print("Emotion requested: "); Serial.println(emo);
    
    if (emo == "neutral") currentEmotion = NEUTRAL;
    else if (emo == "happy") currentEmotion = HAPPY;
    else if (emo == "sad") currentEmotion = SAD;
    else if (emo == "angry") currentEmotion = ANGRY;
    else if (emo == "sleepy") currentEmotion = SLEEPY;
    else if (emo == "surprised") currentEmotion = SURPRISED;
    else if (emo == "crying") currentEmotion = CRYING;
    else if (emo == "wink") currentEmotion = WINK;
    else if (emo == "love") currentEmotion = LOVE;

    // Force an immediate blink/reaction when a new button is pressed
    isBlinking = true; 
    blinkTimer = millis();
  }
  server.send(200, "text/plain", "OK");
}

// ==========================================
// DRAWING ENGINE
// ==========================================
void drawScreen() {
  display.clearDisplay();

  int leftEyeX = 35;
  int rightEyeX = 93;
  int eyeY = 32;

  // WINK ANIMATION LOGIC:
  // If winking, left eye stays open, right eye blinks
  bool leftBlink = isBlinking;
  bool rightBlink = isBlinking;
  
  if (currentEmotion == WINK) {
    leftBlink = false; // Force left eye open
  }

  // Draw Eyes
  drawSingleEye(leftEyeX, eyeY, true, leftBlink);
  drawSingleEye(rightEyeX, eyeY, false, rightBlink);

  // Draw tears if crying
  if (currentEmotion == CRYING) {
    display.fillCircle(leftEyeX - 10, tearDropY, 3, SSD1306_WHITE);
    display.fillCircle(rightEyeX + 10, tearDropY + 5, 3, SSD1306_WHITE); 
  }

  display.display();
}

void drawSingleEye(int x, int y, bool isLeft, bool doBlink) {
  int width = 30;
  int height = 40;

  if (currentEmotion == SURPRISED) {
    width = 34; height = 46;
  } else if (currentEmotion == SLEEPY) {
    height = 20; y += 10;
  }

  if (currentEmotion == LOVE) {
    drawHeart(x, y - 5);
  } else {
    display.fillRoundRect(x - width/2, y - height/2, width, height, 10, SSD1306_WHITE);
    
    if (!doBlink && currentEmotion != SLEEPY && currentEmotion != CRYING) {
      int px = x + lookOffsetX;
      int py = y + lookOffsetY;
      int pSize = (currentEmotion == SURPRISED) ? 4 : 8; 
      display.fillCircle(px, py, pSize, SSD1306_BLACK); 
    }
  }

  // --- MASKS ---
  
  if (doBlink) {
    display.fillRect(x - 20, y - 25, 40, 50, SSD1306_BLACK);
    display.drawLine(x - 15, y, x + 15, y, SSD1306_WHITE); 
    return; 
  }

  if (currentEmotion == HAPPY) {
    display.fillRoundRect(x - 20, y + 5, 40, 20, 5, SSD1306_BLACK);
  } 
  else if (currentEmotion == ANGRY) {
    // FIXED: Angry brows point DOWN towards the nose
    if (isLeft) {
      display.fillTriangle(x - 15, y - 25, x + 20, y - 25, x + 20, y - 5, SSD1306_BLACK);
    } else {
      display.fillTriangle(x - 20, y - 25, x + 15, y - 25, x - 20, y - 5, SSD1306_BLACK);
    }
  } 
  else if (currentEmotion == SAD || currentEmotion == CRYING) {
    // FIXED: Sad brows point UP towards the nose
    if (isLeft) {
      display.fillTriangle(x - 25, y - 25, x + 5, y - 25, x - 25, y - 5, SSD1306_BLACK);
    } else {
      display.fillTriangle(x - 5, y - 25, x + 25, y - 25, x + 25, y - 5, SSD1306_BLACK);
    }
  }
}

void drawHeart(int x, int y) {
  display.fillCircle(x - 7, y, 8, SSD1306_WHITE); 
  display.fillCircle(x + 7, y, 8, SSD1306_WHITE); 
  display.fillTriangle(x - 14, y + 4, x + 14, y + 4, x, y + 18, SSD1306_WHITE); 
}
