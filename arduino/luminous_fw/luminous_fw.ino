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

// Radar ---------------------
struct radarObj_mt {
  int16_t targetX;
  int16_t targetY;
  int16_t targetSpeed;
  uint16_t distResolution;
};

struct radarIn_mt {
  radarObj_mt obj1;
  radarObj_mt obj2;
  radarObj_mt obj3;
  uint8_t end[2];
};

struct radarHandler_mt {
  radarIn_mt radarDataIn;
  uint8_t headerBuffer;
  uint8_t radarBytesRead;
};

radarHandler_mt radar1;
radarHandler_mt radar2;

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

// TODO: Log to display!
void scan() {
  byte error, address;
  int nDevices;

  u8x8.home();

  u8x8.println("I2C Scan:");

  nDevices = 0;
  for (address = 1; address < 127; address++) {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      u8x8.print("+ 0x");
      if (address < 16)
        u8x8.print("0");
      u8x8.println(address, HEX);
      nDevices++;
    } else if (error == 4) {
      u8x8.print("Unknown error at 0x");
      if (address < 16)
        u8x8.print("0");
      u8x8.println(address, HEX);
    }
  }
  if (nDevices == 0)
    u8x8.println("No I2C devices found");
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

  Serial.begin(256000);
  Serial1.begin(256000);

  Wire.begin();

  // Initialize display
  u8x8.begin();  // Address 0x3D for 128x64

  u8x8.setFlipMode(1);
  u8x8.setPowerSave(0);
  u8x8.clearDisplay();
  u8x8.setContrast(255);
  u8x8.setFont(u8x8_font_8x13B_1x2_f);

  scan();
  delay(2000);
  u8x8.clearDisplay();
  u8x8.home();

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

  radar1.headerBuffer = 0xFF;
  radar1.radarBytesRead = 0;

  radar2.headerBuffer = 0xFF;
  radar2.radarBytesRead = 0;

  lastAnimationTime = millis();
}

void processRadarIn(HardwareSerial* hSerial, radarHandler_mt* handler) {
  // We need a fast and efficient algorithm to detect the start of a frame.
  // So here is leo's mathematical algorithmic approach:
  // We take each byte read, negate it, and xor it to our header calculation.
  // Each byte results in our header byte forming a new value.
  // We start with 0xFF. A correct header then is read like the following:
      // header calc new start -> header byte buffer set to 0xFF.
      // UART read 0xAA
      // 0xFF xor ~0xAA = 0b11111111
      //       ^ ~0xAA = 0b01010101
      //               -> 0b10101010 (0xAA)

      // UART read 0xFF
      // 0b10101010 xor ~0xFF = 0b10101010
      //             ^ ~0xFF = 0b00000000
      //                     -> 0b10101010 (0xAA)

      // UART read 0x03
      // 0b10101010 xor ~0x03 = 0b10101010
      //             ^ ~0x03 = 0b11111100
      //                     -> 0b01010110 (0x56)

      // UART read 0x00
      // 0b01010110 xor ~0x00 = 0b01010110
      //             ^ ~0x00 = 0b11111111
      //                     -> 0b10101001 (0xA9)

      // header buffer equals 0xA9 -> header detected.
      // 0xA9 -> Header detected.

      // If our header buffer bytes is neither 0xAA, 0x56 nor 0xA9, we clear our buffer back to 0xFF.

  // This approach does only check three bytes (0xAA,0x03,0x00) of the sensor header. If this leads to false triggering, we may just add one to each read byte and recalculate our expected values.
  // We then have to check against four possible values not just the three above (0xAA is duplicate.)

  uint8_t bytesAvailable = 0;
  while ((bytesAvailable = hSerial->available()) > 0) {

    if (handler->headerBuffer == 0xA9) { // We already found a header. Go read that juicy radar data.
      uint8_t* structPtr = (uint8_t*)&(handler->radarDataIn);
      handler->radarBytesRead += hSerial->readBytes(structPtr + handler->radarBytesRead, min(bytesAvailable, sizeof(radarIn_mt) - handler->radarBytesRead));
      if (handler->radarBytesRead >= sizeof(radarIn_mt)) {
        handler->headerBuffer = 0;
        // process radar data.
        // We finished reading one frame.
      }
    }
    
    else {
      handler->headerBuffer ^= ~hSerial->read();
      if (handler->headerBuffer == 0xA9) {
        // header found.
        // We will continue in the next loop iteration.
      } else if (handler->headerBuffer != 0xAA && handler->headerBuffer != 0x56) {
        // Invalid data. We are out of sync and reset our header buffer variable.
        // Next byte will be treated as (maybe) first of header again.
        handler->headerBuffer = 0xFF;
      }
    }
  }
}


void loop() {
  processRadarIn(&Serial, &radar1);
  processRadarIn(&Serial1, &radar2);

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
