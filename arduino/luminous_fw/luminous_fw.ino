#include <U8x8lib.h>

#include "PCA9635.h"
#include "pins_add.h"

PCA9635 ledArray1(0x0);
PCA9635 ledArray2(0x1);
PCA9635 ledArray3(0x2);
PCA9635 ledArray4(0x3);

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

void setup() {
  setupPins();
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("PCA9635_LIB_VERSION: ");
  Serial.println(PCA9635_LIB_VERSION);
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
  ledArray1.begin(0b00000000, 0b00010100);
  ledArray2.begin(0b00000000, 0b00010100);
  ledArray3.begin(0b00000000, 0b00010100);
  ledArray4.begin(0b00000000, 0b00010100);
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
      uint8_t toggleCommand = lastAnimationState++ == 1 ? PCA963X_LEDON : PCA963X_LEDOFF;
      ledArray1.setLedDriverModeAll(toggleCommand);
      ledArray2.setLedDriverModeAll(toggleCommand);
      ledArray3.setLedDriverModeAll(toggleCommand);
      ledArray4.setLedDriverModeAll(toggleCommand);
      lastAnimationTime = millis();
      lastAnimationState %= 2;
    }
  }

  if (animationMode == 1) {
    if (!animationModeInitialized) {
      ledArray1.setLedDriverModeAll(PCA963X_LEDPWM);
      ledArray2.setLedDriverModeAll(PCA963X_LEDPWM);
      ledArray3.setLedDriverModeAll(PCA963X_LEDPWM);
      ledArray4.setLedDriverModeAll(PCA963X_LEDPWM);
      animationModeInitialized = true;
    }

    uint8_t dutyCycle = int(sin((millis() / 5000.0 * PI * 2)) * 127 + 128);
    for (uint8_t i = 0; i < sizeof(animationData); i++) {
      animationData[i] = dutyCycle;
    }
    ledArray1.writeAll(animationData);
    ledArray2.writeAll(animationData);
    ledArray3.writeAll(animationData);
    ledArray4.writeAll(animationData);
  }

  if (animationMode == 2) {
    if (!animationModeInitialized) {
      animationModeInitialized = true;
    }

    // On/Off, no blinking
    if (millis() - lastAnimationTime > 50) {
      uint8_t toggleCommand = lastAnimationState++ == 1 ? PCA963X_LEDON : PCA963X_LEDOFF;
      ledArray1.setLedDriverModeAll(toggleCommand);
      ledArray2.setLedDriverModeAll(toggleCommand);
      ledArray3.setLedDriverModeAll(toggleCommand);
      ledArray4.setLedDriverModeAll(toggleCommand);
      lastAnimationTime = millis();
      lastAnimationState %= 2;
    }
  }


  // UI-Update requested ---------------------------------
  if (millis() - lastButtonUpdatedTime > 100) {
    lastButtonUpdatedTime = millis();
    int8_t pressedButtonIdx = readButtons();
    if (pressedButtonIdx > -1 && pressedButtonIdx != animationMode) {
      animationMode = pressedButtonIdx;
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







void testWrite1() {
  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Test - write1 - I");
  for (int channel = 0; channel < ledArray1.channelCount(); channel++) {
    for (int pwm = 0; pwm < 256; pwm++) {
      ledArray1.write1(channel, pwm);
    }
  }

  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Test - write 1 - II");
  for (int pwm = 0; pwm < 256; pwm++) {
    for (int channel = 0; channel < ledArray1.channelCount(); channel++) {
      ledArray1.write1(channel, pwm);
    }
  }
}


void testWrite3() {
  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Test - write3 - random RGB");
  for (int channel = 0; channel < (ledArray1.channelCount() - 3); channel++)  // 13 = 16 -3 !!!
  {
    uint8_t R = random(256);
    uint8_t G = random(256);
    uint8_t B = random(256);
    ledArray1.write3(channel, R, G, B);
  }
}


void testWriteN() {
  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Test - writeN ");
  uint8_t arr[16] = { 16, 32, 48, 64, 80, 96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255 };
  ledArray1.writeN(0, arr, 16);  // 16 == ledArray1.channelCount()
}


void testSetGroupPWM_FREQ() {
  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Test - GroupPWM");
  for (int channel = 0; channel < ledArray1.channelCount(); channel++) {
    ledArray1.setLedDriverMode(channel, PCA963X_LEDGRPPWM);
  }
  for (int pwm = 0; pwm < 256; pwm++) {
    ledArray1.setGroupPWM(pwm);
    uint8_t p = ledArray1.getGroupPWM();
    if (p != pwm) {
      Serial.print(millis());
      Serial.print("\t");
      Serial.print("pwm: ");
      Serial.println(pwm);
    }
  }
  ledArray1.setGroupPWM(127);

  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Test - groupFRQ");
  for (int frq = 0; frq < 256; frq++) {
    ledArray1.setGroupFREQ(frq);
    uint8_t f = ledArray1.getGroupFREQ();
    if (f != frq) {
      Serial.print(millis());
      Serial.print("\t");
      Serial.print("frq: ");
      Serial.println(frq);
    }
  }

  // reset to LEDPWM
  for (int channel = 0; channel < ledArray1.channelCount(); channel++) {
    ledArray1.setLedDriverMode(channel, PCA963X_LEDPWM);
  }
}


void testSetAndReadMode() {
  Serial.print(millis());
  Serial.print("\t");
  Serial.println("Test - readMode");

  uint8_t regval = ledArray1.getMode1();
  ledArray1.setMode1(regval);  //  non destructive
  Serial.print(millis());
  Serial.print("\t");
  Serial.print("PCA963X_MODE1: ");
  Serial.println(regval);

  regval = ledArray1.getMode2();
  ledArray1.setMode2(regval);
  Serial.print(millis());
  Serial.print("\t");
  Serial.print("PCA963X_MODE2: ");
  Serial.println(regval);
}

// -- END OF FILE --
