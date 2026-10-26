#include <EEPROM.h>
#include <GyverNTC.h>
#include <Wire.h>
#include <math.h>
#if defined(TEPLOTECH_USE_OLED)
#include <U8g2lib.h>
#ifndef TEPLOTECH_OLED_I2C_ADDRESS
#define TEPLOTECH_OLED_I2C_ADDRESS 0x3C
#endif
#else
#include <LiquidCrystal_I2C.h>
#endif
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// TEPLOTECH EVP-6: local heating controller.
// Relay outputs are active LOW, matching the original wiring.
// Heating is always inhibited at startup until the operator clears STOP.

namespace Pins {
const uint8_t LCD_ADDRESS = 0x27;
#if defined(ARDUINO_ARCH_ESP32)
const uint8_t HEATERS[3] = {25, 26, 27};
const uint8_t PUMP = 14;
const uint8_t KEYPAD = 36;
const uint8_t SENSORS[4] = {32, 33, 34, 35};
const uint8_t I2C_SDA = 21;
const uint8_t I2C_SCL = 22;
#else
const uint8_t HEATERS[3] = {3, 4, 5};
const uint8_t PUMP = 6;
const uint8_t KEYPAD = A3;
const uint8_t SENSORS[4] = {A1, A2, A6, A7};
const uint8_t I2C_SDA = A4;
const uint8_t I2C_SCL = A5;
#endif
}

namespace Limits {
const float MANUAL_TARGET_MIN_C = 0.0f;
const float MANUAL_TARGET_MAX_C = 85.0f;
const float WATER_HARD_LIMIT_C = 85.0f;
const float WATER_HYSTERESIS_C = 2.0f;
const float SENSOR_MIN_C = -50.0f;
const float SENSOR_MAX_C = 125.0f;
const uint32_t SENSOR_READ_INTERVAL_MS = 1000UL;
const uint32_t AUTO_STAGE_INTERVAL_MS = 5000UL;
const uint32_t UI_REFRESH_INTERVAL_MS = 500UL;
const uint32_t BUTTON_DEBOUNCE_MS = 40UL;
const uint16_t EEPROM_MAGIC = 0x5448;
const uint8_t EEPROM_VERSION = 1;
}

enum ControlMode : uint8_t {
  MODE_AUTO = 0,
  MODE_MANUAL = 1
};

enum Screen : uint8_t {
  SCREEN_HOME,
  SCREEN_MENU,
  SCREEN_MODE,
  SCREEN_TARGET,
  SCREEN_HEATERS,
  SCREEN_PUMP
};

enum Button : uint8_t {
  KEY_NONE,
  KEY_LEFT,
  KEY_UP,
  KEY_DOWN,
  KEY_RIGHT,
  KEY_OK
};

struct PersistentSettings {
  uint16_t magic;
  uint8_t version;
  uint8_t mode;
  uint8_t pumpRequested;
  uint8_t manualHeaters[3];
  float manualTargetC;
  uint16_t checksum;
};

struct Readings {
  float roomC;
  float outsideC;
  float waterInC;
  float waterOutC;
  bool roomValid;
  bool outsideValid;
  bool waterInValid;
  bool waterOutValid;
};

// Explicit prototypes keep Arduino IDE sketch preprocessing compatible with
// the user-defined settings and readings types above.
uint16_t crc16Add(uint16_t crc, uint8_t value);
uint16_t settingsChecksum(const PersistentSettings &value);
bool settingsAreValid(const PersistentSettings &value);
void saveSettings();
void loadSettings();
Button readRawButton();
Button readButtonEvent(uint32_t now);
bool readTemperature(uint8_t pin, float &result);
void readAllTemperatures();
float interpolateOutdoorTarget(float outsideC);
float activeTargetC();
void setRelay(uint8_t pin, bool enabled);
void applyOutputs();
void clearHeaterDemand();
void updateHeaterController(uint32_t now);
void printRow(uint8_t row, const char *text);
void renderScreenContents();
void renderHome();
void renderMenu();
void renderScreen();
void enterScreen(Screen next);
void handleMenuButton(Button button);
void handleButton(Button button);
void setup();
void loop();

const int8_t OUTDOOR_POINTS[13] = {
  -40, -37, -35, -30, -25, -20, -15, -10, -5, 0, 5, 8, 10
};
const float WATER_OUT_TARGETS[13] = {
  92.9f, 90.0f, 88.0f, 83.0f, 77.9f, 72.7f, 67.4f,
  61.9f, 56.2f, 50.3f, 44.1f, 40.2f, 37.5f
};

#if defined(TEPLOTECH_USE_OLED)
#if defined(TEPLOTECH_OLED_SH1106)
U8G2_SH1106_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
#else
U8G2_SSD1306_128X64_NONAME_1_HW_I2C oled(U8G2_R0, U8X8_PIN_NONE);
#endif
#else
LiquidCrystal_I2C lcd(Pins::LCD_ADDRESS, 20, 4);
#endif
GyverNTC thermistor;

PersistentSettings settings;
Readings readings = {0, 0, 0, 0, false, false, false, false};
Screen screen = SCREEN_HOME;
bool stopped = true;
bool manualThermostatEnabled = true;
bool heaterDemand[3] = {false, false, false};
bool heaterOutput[3] = {false, false, false};
bool pumpOutput = false;
uint8_t menuSelection = 0;
uint8_t heaterSelection = 0;
uint8_t modeDraft = MODE_AUTO;
uint8_t heaterDraft[3] = {0, 0, 0};
uint8_t pumpDraft = 0;
float targetDraftC = 50.0f;
uint32_t lastSensorReadMs = 0;
uint32_t lastStageMs = 0;
uint32_t lastUiRefreshMs = 0;

uint16_t crc16Add(uint16_t crc, uint8_t value) {
  crc ^= value;
  for (uint8_t bit = 0; bit < 8; ++bit) {
    crc = (crc & 1U) ? (crc >> 1) ^ 0xA001U : crc >> 1;
  }
  return crc;
}

uint16_t settingsChecksum(const PersistentSettings &value) {
  uint16_t crc = 0xFFFFU;
  crc = crc16Add(crc, (uint8_t)(value.magic & 0xFFU));
  crc = crc16Add(crc, (uint8_t)(value.magic >> 8));
  crc = crc16Add(crc, value.version);
  crc = crc16Add(crc, value.mode);
  crc = crc16Add(crc, value.pumpRequested);
  for (uint8_t i = 0; i < 3; ++i) {
    crc = crc16Add(crc, value.manualHeaters[i]);
  }
  const uint8_t *targetBytes = (const uint8_t *)&value.manualTargetC;
  for (uint8_t i = 0; i < sizeof(value.manualTargetC); ++i) {
    crc = crc16Add(crc, targetBytes[i]);
  }
  return crc;
}

bool settingsAreValid(const PersistentSettings &value) {
  if (value.magic != Limits::EEPROM_MAGIC ||
      value.version != Limits::EEPROM_VERSION ||
      value.mode > MODE_MANUAL ||
      value.pumpRequested > 1 ||
      value.manualTargetC < Limits::MANUAL_TARGET_MIN_C ||
      value.manualTargetC > Limits::MANUAL_TARGET_MAX_C ||
      isnan(value.manualTargetC) || isinf(value.manualTargetC) ||
      value.checksum != settingsChecksum(value)) {
    return false;
  }
  for (uint8_t i = 0; i < 3; ++i) {
    if (value.manualHeaters[i] > 1) return false;
  }
  return true;
}

void saveSettings() {
  settings.magic = Limits::EEPROM_MAGIC;
  settings.version = Limits::EEPROM_VERSION;
  settings.checksum = settingsChecksum(settings);
  EEPROM.put(0, settings);
}

void loadSettings() {
  EEPROM.get(0, settings);
  if (settingsAreValid(settings)) return;

  memset(&settings, 0, sizeof(settings));
  settings.magic = Limits::EEPROM_MAGIC;
  settings.version = Limits::EEPROM_VERSION;
  settings.mode = MODE_AUTO;
  settings.pumpRequested = 1;
  settings.manualTargetC = 50.0f;
  saveSettings();
}

Button readRawButton() {
  const int value = analogRead(Pins::KEYPAD);
#if defined(ARDUINO_ARCH_ESP32)
  value = (value * 1023L) / 4095L;
#endif
  if (value < 50) return KEY_LEFT;
  if (value < 160) return KEY_DOWN;
  if (value < 360) return KEY_UP;
  if (value < 560) return KEY_RIGHT;
  if (value < 860) return KEY_OK;
  return KEY_NONE;
}

Button readButtonEvent(uint32_t now) {
  static Button lastRaw = KEY_NONE;
  static Button stable = KEY_NONE;
  static uint32_t changedAt = 0;
  const Button raw = readRawButton();

  if (raw != lastRaw) {
    lastRaw = raw;
    changedAt = now;
  }
  if (now - changedAt < Limits::BUTTON_DEBOUNCE_MS || raw == stable) {
    return KEY_NONE;
  }
  stable = raw;
  return stable;
}

bool readTemperature(uint8_t pin, float &result) {
  thermistor.setPin(pin);
  const float value = thermistor.getTemp();
  if (isnan(value) || isinf(value) ||
      value < Limits::SENSOR_MIN_C || value > Limits::SENSOR_MAX_C) {
    return false;
  }
  result = value;
  return true;
}

void readAllTemperatures() {
  readings.roomValid = readTemperature(Pins::SENSORS[0], readings.roomC);
  readings.outsideValid = readTemperature(Pins::SENSORS[1], readings.outsideC);
  readings.waterInValid = readTemperature(Pins::SENSORS[2], readings.waterInC);
  readings.waterOutValid = readTemperature(Pins::SENSORS[3], readings.waterOutC);
}

float interpolateOutdoorTarget(float outsideC) {
  if (outsideC <= OUTDOOR_POINTS[0]) return WATER_OUT_TARGETS[0];
  if (outsideC >= OUTDOOR_POINTS[12]) return WATER_OUT_TARGETS[12];

  for (uint8_t i = 1; i < 13; ++i) {
    if (outsideC <= OUTDOOR_POINTS[i]) {
      const float x0 = OUTDOOR_POINTS[i - 1];
      const float x1 = OUTDOOR_POINTS[i];
      const float y0 = WATER_OUT_TARGETS[i - 1];
      const float y1 = WATER_OUT_TARGETS[i];
      return y0 + (y1 - y0) * (outsideC - x0) / (x1 - x0);
    }
  }
  return WATER_OUT_TARGETS[12];
}

float activeTargetC() {
  if (settings.mode == MODE_MANUAL) return settings.manualTargetC;
  float target = interpolateOutdoorTarget(readings.outsideC);
  if (target > Limits::WATER_HARD_LIMIT_C) {
    target = Limits::WATER_HARD_LIMIT_C;
  }
  return target;
}

void setRelay(uint8_t pin, bool enabled) {
  digitalWrite(pin, enabled ? LOW : HIGH);
}

void applyOutputs() {
  const bool controlSensorsValid =
      readings.waterOutValid &&
      (settings.mode == MODE_MANUAL || readings.outsideValid);
  const bool mayHeat = !stopped && controlSensorsValid &&
      readings.waterOutC < Limits::WATER_HARD_LIMIT_C;

  bool anyHeaterOn = false;
  for (uint8_t i = 0; i < 3; ++i) {
    const bool enabled = mayHeat && heaterDemand[i];
    if (enabled != heaterOutput[i]) {
      setRelay(Pins::HEATERS[i], enabled);
      heaterOutput[i] = enabled;
    }
    anyHeaterOn = anyHeaterOn || enabled;
  }

  // Keep circulation running whenever a heater is energized.
  const bool shouldRunPump = settings.pumpRequested || anyHeaterOn;
  if (shouldRunPump != pumpOutput) {
    setRelay(Pins::PUMP, shouldRunPump);
    pumpOutput = shouldRunPump;
  }
}

void clearHeaterDemand() {
  for (uint8_t i = 0; i < 3; ++i) heaterDemand[i] = false;
}

void updateHeaterController(uint32_t now) {
  const bool controlSensorsValid =
      readings.waterOutValid &&
      (settings.mode == MODE_MANUAL || readings.outsideValid);
  if (stopped || !controlSensorsValid ||
      readings.waterOutC >= Limits::WATER_HARD_LIMIT_C) {
    clearHeaterDemand();
    applyOutputs();
    return;
  }

  const float target = activeTargetC();
  if (settings.mode == MODE_MANUAL) {
    if (readings.waterOutC >= target) {
      manualThermostatEnabled = false;
    } else if (readings.waterOutC <= target - Limits::WATER_HYSTERESIS_C) {
      manualThermostatEnabled = true;
    }
    for (uint8_t i = 0; i < 3; ++i) {
      heaterDemand[i] = settings.manualHeaters[i] && manualThermostatEnabled;
    }
  } else {
    if (readings.waterOutC >= target) {
      clearHeaterDemand();
    } else if (readings.waterOutC <= target - Limits::WATER_HYSTERESIS_C &&
               now - lastStageMs >= Limits::AUTO_STAGE_INTERVAL_MS) {
      for (uint8_t i = 0; i < 3; ++i) {
        if (!heaterDemand[i]) {
          heaterDemand[i] = true;
          lastStageMs = now;
          break;
        }
      }
    }
  }
  applyOutputs();
}

void printRow(uint8_t row, const char *text) {
#if defined(TEPLOTECH_USE_OLED)
  // Page-buffer U8g2 uses only a small fraction of Nano SRAM.
  oled.drawStr(0, 11 + row * 16, text);
#else
  lcd.setCursor(0, row);
  uint8_t written = 0;
  while (*text && written < 20) {
    lcd.print(*text++);
    ++written;
  }
  while (written++ < 20) lcd.print(' ');
#endif
}

void formatTemperature(char *buffer, size_t size, const char *label,
                       bool valid, float value) {
  if (valid) {
    snprintf(buffer, size, "%s%3dC", label, (int)value);
  } else {
    snprintf(buffer, size, "%s  E", label);
  }
}

void renderHome() {
  char row[21];
  snprintf(row, sizeof(row), "%s %s H:%c%c%c",
           settings.mode == MODE_AUTO ? "AUTO" : "MANUAL",
           stopped ? "STOP" : "RUN",
           heaterOutput[0] ? '1' : '0',
           heaterOutput[1] ? '1' : '0',
           heaterOutput[2] ? '1' : '0');
  printRow(0, row);

  char room[4];
  char outside[4];
  char waterIn[4];
  char waterOut[4];
  if (readings.roomValid) snprintf(room, sizeof(room), "%3d", (int)readings.roomC);
  else strcpy(room, "  E");
  if (readings.outsideValid) snprintf(outside, sizeof(outside), "%3d", (int)readings.outsideC);
  else strcpy(outside, "  E");
  if (readings.waterInValid) snprintf(waterIn, sizeof(waterIn), "%3d", (int)readings.waterInC);
  else strcpy(waterIn, "  E");
  if (readings.waterOutValid) snprintf(waterOut, sizeof(waterOut), "%3d", (int)readings.waterOutC);
  else strcpy(waterOut, "  E");

  snprintf(row, sizeof(row), "Rm:%sC Out:%sC", room, outside);
  printRow(1, row);
  snprintf(row, sizeof(row), "In:%sC Out:%sC", waterIn, waterOut);
  printRow(2, row);

  if (settings.mode == MODE_AUTO && readings.outsideValid) {
    snprintf(row, sizeof(row), "Target:%2dC Pump:%s",
             (int)activeTargetC(), pumpOutput ? "ON" : "OFF");
  } else {
    snprintf(row, sizeof(row), "Set:%2dC Pump:%s",
             (int)settings.manualTargetC, pumpOutput ? "ON" : "OFF");
  }
  printRow(3, row);
}

const char *const MENU_ITEMS[] = {
  "Mode", "Water target", "Heaters", "Pump", "Return"
};

void renderMenu() {
  for (uint8_t row = 0; row < 4; ++row) {
    const uint8_t item = (menuSelection / 4) * 4 + row;
    char text[21];
    if (item < 5) {
      snprintf(text, sizeof(text), "%c %s",
               item == menuSelection ? '>' : ' ', MENU_ITEMS[item]);
    } else {
      text[0] = '\0';
    }
    printRow(row, text);
  }
}

void renderScreenContents() {
  char row[21];
  switch (screen) {
    case SCREEN_HOME:
      renderHome();
      break;
    case SCREEN_MENU:
      renderMenu();
      break;
    case SCREEN_MODE:
      printRow(0, "Operating mode");
      snprintf(row, sizeof(row), "Selected: %s",
               modeDraft == MODE_AUTO ? "AUTO" : "MANUAL");
      printRow(1, row);
      printRow(2, "UP/DOWN change");
      printRow(3, "OK save  LEFT back");
      break;
    case SCREEN_TARGET:
      printRow(0, "Water target");
      snprintf(row, sizeof(row), "Selected: %d C", (int)targetDraftC);
      printRow(1, row);
      printRow(2, "UP +1  DOWN -1");
      printRow(3, "OK save  LEFT back");
      break;
    case SCREEN_HEATERS:
      printRow(0, "Manual heaters");
      for (uint8_t i = 0; i < 3; ++i) {
        snprintf(row, sizeof(row), "%c Heater %c: %s",
                 i == heaterSelection ? '>' : ' ',
                 'A' + i, heaterDraft[i] ? "ON" : "OFF");
        printRow(i + 1, row);
      }
      break;
    case SCREEN_PUMP:
      printRow(0, "Circulation pump");
      snprintf(row, sizeof(row), "Selected: %s", pumpDraft ? "ON" : "OFF");
      printRow(1, row);
      printRow(2, "UP/DOWN change");
      printRow(3, "OK save  LEFT back");
      break;
  }
}

void renderScreen() {
#if defined(TEPLOTECH_USE_OLED)
  oled.firstPage();
  do {
    renderScreenContents();
  } while (oled.nextPage());
#else
  renderScreenContents();
#endif
}

void enterScreen(Screen next) {
  screen = next;
#if !defined(TEPLOTECH_USE_OLED)
  lcd.clear();
#endif
  if (next == SCREEN_MODE) modeDraft = settings.mode;
  if (next == SCREEN_TARGET) targetDraftC = settings.manualTargetC;
  if (next == SCREEN_HEATERS) {
    heaterSelection = 0;
    for (uint8_t i = 0; i < 3; ++i) heaterDraft[i] = settings.manualHeaters[i];
  }
  if (next == SCREEN_PUMP) pumpDraft = settings.pumpRequested;
}

void handleMenuButton(Button button) {
  if (button == KEY_UP) {
    menuSelection = (menuSelection + 4) % 5;
  } else if (button == KEY_DOWN) {
    menuSelection = (menuSelection + 1) % 5;
  } else if (button == KEY_LEFT) {
    enterScreen(SCREEN_HOME);
  } else if (button == KEY_OK) {
    switch (menuSelection) {
      case 0: enterScreen(SCREEN_MODE); break;
      case 1: enterScreen(SCREEN_TARGET); break;
      case 2: enterScreen(SCREEN_HEATERS); break;
      case 3: enterScreen(SCREEN_PUMP); break;
      default: enterScreen(SCREEN_HOME); break;
    }
  }
}

void handleButton(Button button) {
  if (button == KEY_NONE) return;

  if (screen == SCREEN_HOME) {
    if (button == KEY_OK) enterScreen(SCREEN_MENU);
    else if (button == KEY_LEFT) {
      stopped = !stopped;
      clearHeaterDemand();
      applyOutputs();
    } else if (settings.mode == MODE_MANUAL && button == KEY_UP &&
               settings.manualTargetC < Limits::MANUAL_TARGET_MAX_C) {
      settings.manualTargetC += 1.0f;
      saveSettings();
    } else if (settings.mode == MODE_MANUAL && button == KEY_DOWN &&
               settings.manualTargetC > Limits::MANUAL_TARGET_MIN_C) {
      settings.manualTargetC -= 1.0f;
      saveSettings();
    }
    return;
  }

  if (screen == SCREEN_MENU) {
    handleMenuButton(button);
    return;
  }

  if (button == KEY_LEFT) {
    enterScreen(SCREEN_MENU);
    return;
  }

  if (screen == SCREEN_MODE) {
    if (button == KEY_UP || button == KEY_DOWN) {
      modeDraft = modeDraft == MODE_AUTO ? MODE_MANUAL : MODE_AUTO;
    } else if (button == KEY_OK) {
      settings.mode = modeDraft;
      clearHeaterDemand();
      manualThermostatEnabled = true;
      saveSettings();
      enterScreen(SCREEN_MENU);
    }
  } else if (screen == SCREEN_TARGET) {
    if (button == KEY_UP && targetDraftC < Limits::MANUAL_TARGET_MAX_C) {
      targetDraftC += 1.0f;
    } else if (button == KEY_DOWN && targetDraftC > Limits::MANUAL_TARGET_MIN_C) {
      targetDraftC -= 1.0f;
    } else if (button == KEY_OK) {
      settings.manualTargetC = targetDraftC;
      saveSettings();
      enterScreen(SCREEN_MENU);
    }
  } else if (screen == SCREEN_HEATERS) {
    if (button == KEY_UP) {
      heaterSelection = (heaterSelection + 2) % 3;
    } else if (button == KEY_DOWN) {
      heaterSelection = (heaterSelection + 1) % 3;
    } else if (button == KEY_OK) {
      heaterDraft[heaterSelection] = !heaterDraft[heaterSelection];
      for (uint8_t i = 0; i < 3; ++i) {
        settings.manualHeaters[i] = heaterDraft[i];
      }
      saveSettings();
      updateHeaterController(millis());
    }
  } else if (screen == SCREEN_PUMP) {
    if (button == KEY_UP || button == KEY_DOWN) {
      pumpDraft = !pumpDraft;
    } else if (button == KEY_OK) {
      settings.pumpRequested = pumpDraft;
      saveSettings();
      applyOutputs();
      enterScreen(SCREEN_MENU);
    }
  }
}

void setup() {
  // Set inactive relay levels before enabling outputs to avoid a startup pulse.
  for (uint8_t i = 0; i < 3; ++i) {
    digitalWrite(Pins::HEATERS[i], HIGH);
    pinMode(Pins::HEATERS[i], OUTPUT);
  }
  digitalWrite(Pins::PUMP, HIGH);
  pinMode(Pins::PUMP, OUTPUT);

  thermistor.config(10000, 3950);
#if defined(ARDUINO_ARCH_ESP32)
  analogReadResolution(10);
  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);
#else
  Wire.begin();
#endif

#if defined(TEPLOTECH_USE_OLED)
  oled.setI2CAddress((uint8_t)(TEPLOTECH_OLED_I2C_ADDRESS << 1));
  oled.begin();
  oled.setFont(u8g2_font_5x7_tf);
#else
  lcd.init();
  lcd.backlight();
  lcd.clear();
#endif

  loadSettings();
  stopped = true;  // Require an explicit operator action after every reboot.
  lastSensorReadMs = millis() - Limits::SENSOR_READ_INTERVAL_MS;
  lastUiRefreshMs = millis() - Limits::UI_REFRESH_INTERVAL_MS;
}

void loop() {
  const uint32_t now = millis();

  if (now - lastSensorReadMs >= Limits::SENSOR_READ_INTERVAL_MS) {
    lastSensorReadMs = now;
    readAllTemperatures();
    updateHeaterController(now);
  }

  handleButton(readButtonEvent(now));

  if (now - lastUiRefreshMs >= Limits::UI_REFRESH_INTERVAL_MS) {
    lastUiRefreshMs = now;
    renderScreen();
  }
}
