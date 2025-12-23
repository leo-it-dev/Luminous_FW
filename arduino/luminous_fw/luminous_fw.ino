#include <U8x8lib.h>

#include "PCA9685.h"
#include "pins_add.h"

PCA9685 ledArray4(PCA_ADD_HP);
PCA9685 ledArray1(PCA_ADD_LP_0);
PCA9685 ledArray2(PCA_ADD_LP_1);
PCA9685 ledArray3(PCA_ADD_LP_2);

U8X8_SH1106_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);

#define BUTTON_COUNT 3
#define BUTTON_THRESHOLD_INTERVAL (255 / BUTTON_COUNT)
#define BUTTON_THRESHOLD(n) (BUTTON_THRESHOLD_INTERVAL / 2 + BUTTON_THRESHOLD_INTERVAL*(n))

uint8_t animationMode = 0;
bool animationModeInitialized = false;

// Animation data ------------------------------------------------
// Animation-mode-0 ---------------
long lastAnimationTime = 0;
uint8_t lastAnimationState = 0;
// Animation-mode-1 ---------------
uint8_t animationData[16] = {0};
// ---------------------------------------------------------------

// UI-Subroutine -------------
bool uiUpdatedRequested = true;
long lastButtonUpdatedTime = 0;
// ---------------------------

void setupPins() {
  pinMode(PCA_ENABLE_HP_PIN, OUTPUT);
  digitalWrite(PCA_ENABLE_HP_PIN, LOW);
  pinMode(PCA_ENABLE_LP_PIN, OUTPUT);
  digitalWrite(PCA_ENABLE_LP_PIN, LOW);

  pinMode(RADAR_PIN, INPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(NTC_ADC_PIN, INPUT);
  pinMode(BTN_ADC_PIN, INPUT);
  pinMode(LDR1_PIN, INPUT);
  pinMode(LDR2_PIN, INPUT);
  pinMode(BTN_REF_PIN, INPUT);
  pinMode(STEP_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);
  pinMode(ESP_EN_PIN, OUTPUT);
  digitalWrite(ESP_EN_PIN, ESP_RUN);
  pinMode(ESP_RESET_PIN, OUTPUT);
  digitalWrite(ESP_RESET_PIN, ESP_RUN);
  pinMode(DMX_DIR_PIN, OUTPUT);
  digitalWrite(DMX_DIR_PIN, DMX_DIR_READ);
}

void scan() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    } else if (error == 4) {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

int8_t readButtons() {
  uint8_t btnAnalog = analogRead(BTN_ADC_PIN);
  int8_t buttonPressed = int(round(float(btnAnalog) / BUTTON_THRESHOLD_INTERVAL)) - 1;
  return buttonPressed;
}


// HSV->RGB conversion based on GLSL version
// expects hsv channels defined in 0.0 .. 1.0 interval
float fract(float x) { return x - int(x); }
float mix(float a, float b, float t) { return a + (b - a) * t; }

float* hsv2rgb(float h, float s, float b, float* rgb) {
  rgb[0] = b * mix(1.0, constrain(abs(fract(h + 1.0) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
  rgb[1] = b * mix(1.0, constrain(abs(fract(h + 0.6666666) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
  rgb[2] = b * mix(1.0, constrain(abs(fract(h + 0.3333333) * 6.0 - 3.0) - 1.0, 0.0, 1.0), s);
  return rgb;
}

void setup() {
  setupPins();
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.println();

  Wire.begin();
  scan();

  // Initialize display
  u8x8.begin();  // Address 0x3D for 128x64

  u8x8.setFlipMode(1);
  u8x8.setPowerSave(0);
  u8x8.clearDisplay();
  u8x8.setContrast(255);
  u8x8.setFont(u8x8_font_8x13B_1x2_f);

  // LED PCA9635 init
  ledArray1.begin(0b00100000, 0b00010100);
  ledArray2.begin(0b00100000, 0b00010100);
  ledArray3.begin(0b00100000, 0b00010100);
  ledArray4.begin(0b00100000, 0b00010100);

  ledArray1.setFrequency(500, 0); // 500Hz pwm frequency, no phase offset
  ledArray2.setFrequency(500, 0);
  ledArray3.setFrequency(500, 0);
  ledArray4.setFrequency(500, 0);

  ledArray1.setOutputEnablePin(PCA_ENABLE_LP_PIN);
  ledArray4.setOutputEnablePin(PCA_ENABLE_HP_PIN);
  ledArray1.setOutputEnable(true);
  ledArray4.setOutputEnable(true);

  lastAnimationTime = millis();
}

void loop() {

  // Animation modes ---------------------------------
  if (animationMode == 0) {
    if (!animationModeInitialized) {
      animationModeInitialized = true;
    }

    // On/Off, no blinking
    if (millis() - lastAnimationTime > 1000) {
      if (lastAnimationState++ == 1) {
        ledArray1.allON();
        ledArray2.allON();
        ledArray3.allON();
        ledArray4.allON();
      } else {
        ledArray1.allOFF();
        ledArray2.allOFF();
        ledArray3.allOFF();
        ledArray4.allOFF();
      }

      lastAnimationTime = millis();
      lastAnimationState %= 2;
    }
  }

  if (animationMode == 1) {
    if (!animationModeInitialized) {
      animationModeInitialized = true;
    }

    uint16_t dutyCycle16 = abs(int(sin((millis() / 8000.0 * PI * 2)) * 4096));
    float colors[] = {0.0, 0.0, 0.0, 1.0};
    float h = ((millis() % 2000) / 2000.0);
    hsv2rgb(h, 1, 1, colors);

    for (uint8_t channel = 0; channel < 16; channel++) {
      uint16_t color = (uint16_t)(colors[channel % (sizeof(colors) / sizeof(float))] * 0xFFF);
      ledArray1.setPWM(channel, color);
      ledArray2.setPWM(channel, color);
      ledArray3.setPWM(channel, color);
    }

    ledArray4.setPWMAll(dutyCycle16);
  }

  if (animationMode == 2) {
    if (!animationModeInitialized) {
      animationModeInitialized = true;
    }

    // On/Off, no blinking
    if (millis() - lastAnimationTime > 50) {
      if (lastAnimationState++ == 1) {
        ledArray1.allON();
        ledArray2.allON();
        ledArray3.allON();
        ledArray4.allON();
      } else {
        ledArray1.allOFF();
        ledArray2.allOFF();
        ledArray3.allOFF();
        ledArray4.allOFF();
      }

      lastAnimationTime = millis();
      lastAnimationState %= 2;
    }
  }

  if (animationMode == 3) {
    if (!animationModeInitialized) {
      animationModeInitialized = true;
    }

    for (uint8_t i = 0; i < sizeof(animationData); i++) {
      animationData[i] = 0;
    }

    if (digitalRead(LDR1_PIN)) {
      animationData[3] = 255;
    }
    if (digitalRead(LDR2_PIN)) {
      animationData[1] = 255;
      animationData[0] = 255;
    }
    if (digitalRead(PIR_PIN)) {
      animationData[2] = 255;
    }

    // TODO: Implement
    // ledArray1.writeAll(animationData);
  }

  if (animationMode == 4) {
    if (!animationModeInitialized) {
      animationModeInitialized = true;
    }

    for (uint8_t i = 0; i < sizeof(animationData); i++) {
      animationData[i] = 0;
    }

    uint16_t radarIn = analogRead(RADAR_PIN);

    animationData[0] = 255;
    animationData[1] = 255;

    if (radarIn > 800) { // 1024, Idle
      // Idle
    } else if (radarIn > 600) { // 696, Rad1 triggered
      animationData[3] = 255;
    } else if (radarIn > 450) { // 512, Rad2 triggered
      animationData[2] = 255;
    } else if (radarIn > 300) { // 410, Both triggered
      animationData[3] = 255;
      animationData[2] = 255;
    }

    // ledArray1.writeAll(animationData);
  }


  // UI-Update requested ---------------------------------
  if (millis() - lastButtonUpdatedTime > 100) {
    lastButtonUpdatedTime = millis();
    uint8_t lastAnimationMode = animationMode;
    int8_t pressedButtonIdx = readButtons();
    if (pressedButtonIdx == 0 && animationMode > 0) {
      animationMode--;
    }
    if (pressedButtonIdx == 2 && animationMode < 4) {
      animationMode++;
    }
    if (animationMode != lastAnimationMode) {
      animationModeInitialized = false;
      uiUpdatedRequested = true;
      u8x8.clearDisplay();
    }
  }
  if (uiUpdatedRequested) {
    u8x8.home();
    u8x8.print("Anim Mode: ");
    u8x8.println(animationMode);
    u8x8.flush();
  }
}


// -- END OF FILE --
