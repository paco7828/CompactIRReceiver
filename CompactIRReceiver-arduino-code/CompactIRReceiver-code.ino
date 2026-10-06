
#include <IRrecv.h>
#include <IRutils.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SPI.h>
#include <SD.h>

// Buttons
struct Button {
  int pin;
  int lastReading;
  int stableState;
  unsigned long lastChangeTime;
};
Button btnA = { 1, HIGH, HIGH, 0 };
Button btnB = { 2, HIGH, HIGH, 0 };
const uint8_t DEBOUNCE_DELAY = 50;

// Display
const uint8_t I2C_SDA = 8;
const uint8_t I2C_SCL = 9;
const uint8_t I2C_DISPLAY_ADDR = 0x3C;
const uint8_t SCREEN_WIDTH = 128;
const uint8_t SCREEN_HEIGHT = 32;
const uint8_t OLED_RESET = -1;
const char *MENU_TITLE = "IR logger";

// SD Card
const uint8_t SD_CS = 3;
const uint8_t SD_MOSI = 4;
const uint8_t SD_MISO = 10;
const uint8_t SD_SCK = 7;
const char *NO_SD_FEEDBACK = "No SD card";
bool sdAvailable = false;

// IR signal
const uint8_t IR_PIN = 0;
const int BUF_SIZE = 1024;
const uint8_t IR_TIMEOUT = 15;
const uint8_t MIN_UNKNOWN = 12;

uint64_t capturedValue = 0;
uint16_t capturedRawLen = 0;
uint8_t capturedBytes = 0;
uint8_t signalCount = 0;

// Animation
uint8_t dotCount = 0;
uint8_t dotDirection = 1;
unsigned long lastDotTime = 0;
unsigned long feedbackTime = 0;
const int DOT_INTERVAL = 450;
const int FEEDBACK_DURATION = 1500;

// Modes
const uint8_t MODE_MENU = 0;
const uint8_t MODE_LISTENING = 1;
const uint8_t MODE_CAPTURED = 2;
const uint8_t MODE_SAVED = 3;
const uint8_t MODE_DISCARDED = 4;
uint8_t currentMode = 0;

// Instances
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
IRrecv irrecv(IR_PIN, BUF_SIZE, IR_TIMEOUT, true);
decode_results results;

// Function prorotypes
bool debounceBtn(Button &btn);
void showMenu();
void showListening(uint8_t dots);
void showCaptured();
void drawCheck(uint8_t x, uint8_t y, uint8_t size);
void drawCross(uint8_t x, uint8_t y, uint8_t size);
void showFeedback(bool saved);
int getNextFileIndex();
void displaySDError();
bool saveToSD(int idx);

void setup() {
  Serial.begin(115200);

  // Display I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, I2C_DISPLAY_ADDR)) {
    while (1)
      ;
  }

  // SD SPI
  SPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
  sdAvailable = SD.begin(SD_CS);
  if (!sdAvailable) {
    displaySDError();
    delay(1500);
  }

  showMenu();
  irrecv.setUnknownThreshold(MIN_UNKNOWN);
  irrecv.enableIRIn();

  pinMode(btnA.pin, INPUT_PULLUP);
  pinMode(btnB.pin, INPUT_PULLUP);
}

// LOOP
void loop() {
  bool pressA = debounceBtn(btnA);
  bool pressB = debounceBtn(btnB);

  switch (currentMode) {

    case MODE_MENU:
      if (pressA) {
        dotCount = 0;
        dotDirection = 1;
        lastDotTime = millis();
        currentMode = MODE_LISTENING;
        irrecv.resume();
        showListening(dotCount);
      }
      break;

    case MODE_LISTENING:

      if (pressA || pressB) {
        currentMode = MODE_MENU;
        showMenu();
        break;
      }

      if (millis() - lastDotTime >= DOT_INTERVAL) {

        dotCount += dotDirection;

        if (dotCount >= 3) {
          dotCount = 3;
          dotDirection = -1;
        } else if (dotCount <= 0) {
          dotCount = 0;
          dotDirection = 1;
        }

        lastDotTime = millis();
        showListening(dotCount);
      }

      if (irrecv.decode(&results)) {
        if (results.decode_type != UNKNOWN && !results.repeat) {
          capturedValue = results.value;
          capturedBytes = (results.bits + 7) / 8;
          capturedRawLen = results.rawlen;
          currentMode = MODE_CAPTURED;
          showCaptured();
        } else {
          irrecv.resume();
        }
      }
      break;

    case MODE_CAPTURED:
      if (pressA || pressB) {
        // SD card OK & Button A pressed
        if (sdAvailable && pressA) {
          // Save signal on SD card
          signalCount++;
          int idx = getNextFileIndex();
          saveToSD(idx);
          feedbackTime = millis();
          currentMode = MODE_SAVED;
          showFeedback(true);
        } else {
          // SD card FAIL (only discard is available)
          feedbackTime = millis();
          currentMode = MODE_DISCARDED;
          showFeedback(false);
        }
      }
      break;

    case MODE_SAVED:
    case MODE_DISCARDED:
      if (millis() - feedbackTime >= FEEDBACK_DURATION) {
        currentMode = MODE_MENU;
        showMenu();
      }
      break;
  }

  delay(10);
}

void displaySDError() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(NO_SD_FEEDBACK, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2);
  display.print(NO_SD_FEEDBACK);
  display.display();
}

bool saveToSD(int idx) {
  char filename[32];
  snprintf(filename, sizeof(filename), "/Signal-%d.txt", idx);

  File file = SD.open(filename, FILE_WRITE);
  if (!file) return false;

  // HEX
  file.print(capturedValue, HEX);
  file.print(";");

  // Raw length
  file.print(capturedRawLen);
  file.print(";{");

  for (int i = 1; i < capturedRawLen; i++) {

    uint32_t val = results.rawbuf[i];

    file.print(val);

    if (i < capturedRawLen - 1) {
      file.print(",");
    }
  }

  file.println("}");
  file.close();
  return true;
}

int getNextFileIndex() {
  int i = 1;
  char path[32];

  while (true) {
    snprintf(path, sizeof(path), "/Signal-%d.txt", i);
    if (!SD.exists(path)) return i;
    i++;
  }
}

// debounce
bool debounceBtn(Button &btn) {
  int reading = digitalRead(btn.pin);

  if (reading != btn.lastReading) {
    btn.lastChangeTime = millis();
  }

  btn.lastReading = reading;

  if (millis() - btn.lastChangeTime > DEBOUNCE_DELAY) {
    if (btn.stableState != reading) {
      btn.stableState = reading;
      if (btn.stableState == LOW) return true;
    }
  }

  return false;
}

void showMenu() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(MENU_TITLE, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2);
  display.print(MENU_TITLE);
  display.display();
}

void showListening(uint8_t dots) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds("Listening", 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 4);
  display.print("Listening");

  // 3 animated dot circles centered
  const uint8_t DOT_R = 3;
  const uint8_t DOT_GAP = 12;
  const uint8_t DOT_Y = 24;
  uint8_t startX = (SCREEN_WIDTH - (2 * DOT_GAP)) / 2;

  for (uint8_t i = 0; i < 3; i++) {
    uint8_t cx = startX + i * DOT_GAP;
    if (i < dots)
      display.fillCircle(cx, DOT_Y, DOT_R, SSD1306_WHITE);
    else
      display.drawCircle(cx, DOT_Y, DOT_R, SSD1306_WHITE);
  }
  display.display();
}

void showCaptured() {
  char hexStr[20];
  snprintf(hexStr, sizeof(hexStr), "0x%0*llX", capturedBytes * 2, capturedValue);

  char lenStr[12];
  snprintf(lenStr, sizeof(lenStr), "Size: %d", capturedRawLen);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  int16_t x1, y1;
  uint16_t w, h;

  // Hex value top-centered
  display.setTextSize(2);
  display.getTextBounds(hexStr, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 0);
  display.print(hexStr);

  // Signal length
  display.setTextSize(1);
  display.getTextBounds(lenStr, 0, 0, &x1, &y1, &w, &h);
  display.setCursor((SCREEN_WIDTH - w) / 2, 18);
  display.print(lenStr);

  // Bottom icons
  const uint8_t ICON_SIZE = 7;
  const uint8_t ICON_Y = 24;

  if (sdAvailable) {
    drawCheck(6, ICON_Y, ICON_SIZE);
  } else {
    drawCross(6, ICON_Y, ICON_SIZE);
  }
  drawCross(SCREEN_WIDTH - ICON_SIZE - 6, ICON_Y, ICON_SIZE);

  display.display();
}

// Draw checkmark
void drawCheck(uint8_t x, uint8_t y, uint8_t size) {
  // Two-pixel thick check mark scaled to size parameter
  uint8_t mx = x + size * 0.35;
  uint8_t my = y + size;
  display.drawLine(x, y + size * 0.6, mx, my, SSD1306_WHITE);
  display.drawLine(mx, my, x + size, y, SSD1306_WHITE);
  display.drawLine(x, y + size * 0.6 + 1, mx, my + 1, SSD1306_WHITE);
  display.drawLine(mx, my + 1, x + size, y + 1, SSD1306_WHITE);
}

// Draw 'X'
void drawCross(uint8_t x, uint8_t y, uint8_t size) {
  display.drawLine(x, y, x + size, y + size, SSD1306_WHITE);
  display.drawLine(x + size, y, x, y + size, SSD1306_WHITE);
  display.drawLine(x + 1, y, x + size + 1, y + size, SSD1306_WHITE);
  display.drawLine(x + size + 1, y, x + 1, y + size, SSD1306_WHITE);
}

// Display feedback message based on parameter
void showFeedback(bool saved) {
  const char *text = saved ? "Saved" : "Cancel";

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);

  int16_t x1, y1;
  uint16_t tw, th;

  display.getTextBounds(text, 0, 0, &x1, &y1, &tw, &th);

  const uint8_t ICON_SIZE = 10;
  const uint8_t GAP = 6;

  // Text & icon width
  uint16_t totalW = tw + GAP + ICON_SIZE;

  uint8_t startX = (SCREEN_WIDTH - totalW) / 2;
  uint8_t textY = (SCREEN_HEIGHT - th) / 2;
  uint8_t iconY = textY + 2;

  // Draw text
  display.setCursor(startX, textY);
  display.print(text);

  // Draw icon after text
  uint8_t iconX = startX + tw + GAP;

  if (saved)
    drawCheck(iconX, iconY, ICON_SIZE);
  else
    drawCross(iconX, iconY, ICON_SIZE);

  display.display();
}
