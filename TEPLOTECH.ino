#define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
#define DEBUG(label, val) \
  Serial.print(label);    \
  Serial.print(": ");     \
  Serial.println(val)
#else
#define DEBUG(label, val)
#endif

#include <EEPROM.h>
#include <OneWire.h>
#include <Wire.h>
#include <GyverNTC.h>
#include "SPI.h"
#include "Ethernet.h"
#include <LiquidCrystal_I2C.h>
#include <avr/wdt.h>
#include "GyverTimer.h"

void (*resetFunc)(void) = 0; // Функция перезагрузки

struct MenuConstants
{
  static const int MAX_MENU_ITEMS_DISPLAYED = 4;
  static const int LEFT = 1;
  static const int UP = 2;
  static const int DOWN = 3;
  static const int RIGHT = 4;
  static const int OK = 5;
};

struct PinConstants
{
  static const int TEN_PINS[3];
  static const int TEMP_PINS[4];
  static const int LCD_ADDRESS = 0x27;
  static const int PUMP_PIN = 6;
  static const int KeyPadPin = A3;
};

const int PinConstants::TEN_PINS[3] = {3, 4, 5};
const int PinConstants::TEMP_PINS[4] = {A1, A2, A6, A7};

struct DataConstants
{
  static const int B_VALUE = 3950;
  static const int NOMINAL_TEMPERATURE = 25;
  static const int NOMINAL_RESISTANCE = 10000;
  static const int REFERENCE_RESISTANCE = 10000;
  static const int SERIAL_BAUD_RATE = 9600;
  static const int MAX_HEATERS = 3;
  static const int numPoints = 13;
  static const int startAddress = 0;
  static const int NUM_TEMP_SENSORS = 4;
  static const int MAX_MENU_ITEMS_DISPLAYED = 4;
};

struct CharsConstants
{
  static const uint8_t MARKER[8];
  static const uint8_t STOP[8];
  static const uint8_t TEMP_OUT[8];
  static const uint8_t TEMP_IN[8];
  static const uint8_t PUMP[8];
  static const uint8_t EVP[8];
};

struct OtherConstants
{
  static const unsigned long updateInterval = 1500;
  static const int debounceTime = 100;
  static const int thermReadWindowInterval = 50000;
  static const int lcdUpdateInterval = 650;
};

// Инициализация массивов констант

const uint8_t CharsConstants::MARKER[8] = {0x10, 0x18, 0x1C, 0x1E, 0x1C, 0x18, 0x10};
const uint8_t CharsConstants::STOP[8] = {0x00, 0x1B, 0x1B, 0x1B, 0x1B, 0x1B, 0x1B, 0x00};
const uint8_t CharsConstants::TEMP_OUT[8] = {0x04, 0x0A, 0x0A, 0x0A, 0x0A, 0x11, 0x1F, 0x0E};
const uint8_t CharsConstants::TEMP_IN[8] = {0x04, 0x0A, 0x0A, 0x0E, 0x0E, 0x1F, 0x1F, 0x0E};
const uint8_t CharsConstants::PUMP[8] = {0x0A, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x0A, 0x0A};
const uint8_t CharsConstants::EVP[8] = {0x1C, 0x1F, 0x11, 0x11, 0x15, 0x11, 0x1F, 0x1C};

struct TemperatureData
{
  static const int numPoints = 13;
  static const int t_n[numPoints];
  static const float t_wo[numPoints];
  static const float t_wi[numPoints];
  float insideTemp, outsideTemp, waterInTemp, waterOutTemp, t_wi_output, t_wo_output;
};

struct MenuItemData
{
  unsigned int currentMenuItem = 0; // Currently selected option
};

struct TimeData
{
  unsigned long currentTime, loopTime, lastActionTime, lastmillis, lastTempActionTime;
  unsigned long lastUpdateTime = 0;
  unsigned long updateInterval = 1500; // Update every 1.5 seconds
};

MenuItemData menuItemData;
TimeData timeData;
TemperatureData temperatureData;

const int TemperatureData::t_n[numPoints] = {-40, -37, -35, -30, -25, -20, -15, -10, -5, 0, 5, 8, 10};
const float TemperatureData::t_wo[numPoints] = {92.9, 90.0, 88, 83, 77.9, 72.7, 67.4, 61.9, 56.2, 50.3, 44.1, 40.2, 37.5};
const float TemperatureData::t_wi[numPoints] = {71.9, 70.0, 68, 63, 57.9, 52.7, 47.4, 41.9, 36.2, 30.3, 24.1, 20.2, 17.5};

struct MenuItem
{
  const char *label;
  void (*action)();
  MenuItem *subMenu; // Добавлено поле для хранения субменю
};

typedef MenuItem SubMenu;

struct NetworkSettings
{
  String ipAddress;
  String subnetMask;
  String gateway;
};

byte readKey()
{
  static unsigned long lastPressTime = 0; // Время последнего нажатия
  int val = analogRead(PinConstants::KeyPadPin);
  byte currentKey = 0;
  if (val < 50)
    // 1  left
    currentKey = MenuConstants::LEFT;
  else if (val < 160)
    // 2 160 up
    currentKey = MenuConstants::DOWN;
  else if (val < 360)
    // 3 360 down
    currentKey = MenuConstants::UP;
  else if (val < 560)
    // 4 560 right
    currentKey = MenuConstants::RIGHT;
  else if (val < 860)
    // 5 860 menu
    currentKey = MenuConstants::OK;
  unsigned long currentTime = millis();
  if (currentTime - lastPressTime >= 100)
  {
    lastPressTime = currentTime;
    return currentKey;
  }
  return 0;
}

float interpolate(float x, int numPoints, int xData[], float yData[])
{
  if (x <= xData[0])
    return yData[0];
  if (x >= xData[numPoints - 1])
    return yData[numPoints - 1];

  for (int i = 1; i < numPoints; i++)
  {
    if (x < xData[i])
    {
      float x0 = xData[i - 1];
      float x1 = xData[i];
      float y0 = yData[i - 1];
      float y1 = yData[i];
      return y0 + (y1 - y0) * (x - x0) / (x1 - x0);
    }
  }
  return 0;
}

enum MenuState
{
  NORMAL,
  MAIN_MENU,
  SUB_MENU,
  CH_SYSTEM_STATUS,
  CH_HEATERS_STATUS,
  CH_PUMP_STATUS,
  CH_WORK_MODE,
  SH_SYSTEM_SETTINGS,

};

enum SubMenuState
{
  NONE,
  SYSTEM_SETTINGS,
  NETWORK_SETTINGS,
  WOUT_SETUP,
  WIN_SETUP,
  IN_TEMP_SETUP,
};

enum WorkMode
{
  MAN_MODE = 0,
  HS_MODE = 1,
}; // Предполагаемый перечислимый тип WorkMode
enum HeaterState
{
  OFF = 0,
  ON = 1,
}; // Предполагаемый перечислимый тип HeaterState

WorkMode workMode = MANUAL;
MenuState menuState = NORMAL;
SubMenuState subMenuState = NONE;

struct SystemState
{
  int currentMenuItem;
  float setWoInTemp;
  float setRoomTemp;
  int workMode;
  int pumpState;
  int stopState;
  int heater[3];
  int tempItem;
  int selectedItem;
  int NetworkEditMode;
  bool tempReached; // Флаг достижения температуры
  int tempHeater[3];
  NetworkSettings settings;
  MenuState menuState;
  SubMenuState subMenuState;
  SubMenu *subMenu;
  SubMenu *previousSubMenu;
  MenuItem *currentSubMenu;

};

SystemState systemState;

struct EEPROMData
{
  float setRoomTemp;
  float setWoInTemp;
  int workMode;
  int pumpState;
  int stopState;
  int heater[3];
  String ipAddress;
  String subnetMask;
  String gateway;

  // Функция для обновления значений переменных в структуре из данных SystemState
  void updateFromSystemState(const SystemState &systemState)
  {
    setRoomTemp = systemState.setRoomTemp;
    setWoInTemp = systemState.setWoInTemp;
    workMode = systemState.workMode;
    pumpState = systemState.pumpState;
    pumpState = systemState.stopState;
    ipAddress = systemState.settings.ipAddress;
    subnetMask = systemState.settings.subnetMask;
    gateway = systemState.settings.gateway;
    for (int i = 0; i < 3; ++i)
    {
      heater[i] = systemState.heater[i];
    }
  }

  void writeToEEPROM(int startAddress)
  {
    EEPROM.put(startAddress, *this);
  }

  void readFromEEPROM(int startAddress)
  {
    EEPROM.get(startAddress, *this);
  }

  bool isEEPROMInitialized(int startAddress)
  {
    int value = 0;
    for (int i = startAddress; i < sizeof(EEPROMData); ++i)
    {
      value += EEPROM.read(i);
    }
    return value != 0; // Если значение не равно 0, значит, в EEPROM есть данные
  }
  void clearEEPROM(int startAddress)
  {
    for (unsigned int i = 0; i < sizeof(EEPROMData); ++i)
    {
      EEPROM.write(startAddress + i, 0xFF); // Записать каждый байт в EEPROM значением 0xFF
    }
  }
};

EEPROMData eepromData;

SubMenu subMenuNetworkSettings[] = {
    {"IP Setup", nullptr, nullptr},
    {"Mask Setup", nullptr, nullptr},
    {"GW Setup", nullptr, nullptr}};

SubMenu subMenuSettings[] = {
    {
        "Wout Setup",
        nullptr,
    },
    {"Network Setup", []()
     {
       systemState.menuState = SUB_MENU;
       systemState.subMenuState = NETWORK_SETTINGS;
     },
     nullptr},
    {"Reset System", []()
     {
       eepromData.clearEEPROM(DataConstants::startAddress);
       resetFunc(); // Программная перезагрузка Arduino после стирания EEPROM
     },
     nullptr}};

// Объединение всех субменю в один массив
SubMenu allSubMenus[] = {
    {"Heaters On/Off", []()
     {
       systemState.menuState = CH_HEATERS_STATUS;
     },
     nullptr},
    {"Pump On/Off", []()
     {
       systemState.menuState = CH_PUMP_STATUS;
     },
     nullptr},
    {"Work mode", []()
     {
       systemState.menuState = CH_WORK_MODE;
     },
     nullptr},
    {"Settings", []()
     {
       systemState.menuState = SUB_MENU;
       systemState.subMenuState = SYSTEM_SETTINGS;
     },
     subMenuSettings}, // Добавьте другие подменю, если они есть
};

MenuItem menuItems[sizeof(allSubMenus) / sizeof(allSubMenus[0])] = {0}; // Создание меню на основе количества элементов массива allSubMenus

void setupMenu()
{
  for (int i = 0; i < sizeof(allSubMenus) / sizeof(allSubMenus[0]); ++i)
  {
    menuItems[i] = allSubMenus[i];
  }
}

LiquidCrystal_I2C lcd(PinConstants::LCD_ADDRESS, 20, 4);
GyverNTC therm;
GTimer LCDUpdater(MS, 1000);
GTimer ThermReadWindow(MS, 50000);

void setup()
{
#ifdef DEBUG_ENABLE
  Serial.begin(DataConstants::SERIAL_BAUD_RATE);
#endif
  timeData.currentTime = millis();
  timeData.loopTime = timeData.currentTime;
  therm.config(DataConstants::REFERENCE_RESISTANCE, DataConstants::B_VALUE);

  if (!eepromData.isEEPROMInitialized(DataConstants::startAddress))
  {
    // Инициализация значений по умолчанию, если EEPROM пуста
    initializeDefaultSettings();
  }
  else
  {
    // Чтение сохраненных значений из EEPROM
    readSettingsFromEEPROM();
  }

  initializeHardware();
  initializeLCD();
  systemState.currentMenuItem = 0;
  systemState.menuState = NORMAL;
  systemState.subMenuState = NULL;
  systemState.tempItem = 0;
}

void initializeDefaultSettings()
{
  SystemState defaultSystemState;
  defaultSystemState.setRoomTemp = 25.0;
  defaultSystemState.setWoInTemp = 0.0;
  defaultSystemState.workMode = WorkMode::MAN_MODE;
  defaultSystemState.pumpState = HeaterState::ON;
  defaultSystemState.stopState = HeaterState::ON;
  defaultSystemState.heater[0] = HeaterState::OFF;
  defaultSystemState.heater[1] = HeaterState::OFF;
  defaultSystemState.heater[2] = HeaterState::OFF;
  defaultSystemState.tempItem = 0;
  defaultSystemState.selectedItem = 0;
  defaultSystemState.NetworkEditMode = 0;
  defaultSystemState.tempReached = false;
  defaultSystemState.tempHeater[0] = HeaterState::OFF;
  defaultSystemState.tempHeater[1] = HeaterState::OFF;
  defaultSystemState.tempHeater[2] = HeaterState::OFF;
  defaultSystemState.settings.ipAddress = "192.168.000.200";
  defaultSystemState.settings.subnetMask = "255.255.255.000";
  defaultSystemState.settings.gateway = "192.168.000.001";
  defaultSystemState.menuState = MenuState::NORMAL;

  eepromData.updateFromSystemState(defaultSystemState);
  eepromData.writeToEEPROM(DataConstants::startAddress);
}

void readSettingsFromEEPROM()
{
  // Чтение сохраненных значений из EEPROM
  // ...
  eepromData.readFromEEPROM(DataConstants::startAddress);
  systemState.setRoomTemp = eepromData.setRoomTemp;
  systemState.setWoInTemp = eepromData.setWoInTemp;
  systemState.workMode = eepromData.workMode;
  systemState.pumpState = eepromData.pumpState;
  systemState.stopState = eepromData.stopState;
  systemState.settings.ipAddress = eepromData.ipAddress;
  systemState.settings.subnetMask = eepromData.subnetMask;
  systemState.settings.gateway = eepromData.gateway;
  for (int i = 0; i < DataConstants::MAX_HEATERS; ++i)
  {
    systemState.heater[i] = eepromData.heater[i];
    systemState.tempHeater[i] = systemState.heater[i];
  }
}

void initializeHardware()
{
  // Инициализация пинов для нагревателей и насоса
  // ...
  // pinMode(Pins::PUMP, OUTPUT);
  for (int i = 0; i < 3; i++)
  {
    pinMode(PinConstants::TEN_PINS[i], OUTPUT);
    digitalWrite(PinConstants::TEN_PINS[i], LOW);
    digitalWrite(PinConstants::TEN_PINS[i], HIGH);
  }
  pinMode(PinConstants::PUMP_PIN, OUTPUT);
  digitalWrite(PinConstants::PUMP_PIN, LOW);
  digitalWrite(PinConstants::PUMP_PIN, HIGH);
}

void initializeLCD()
{
  // Инициализация LCD и пользовательских символов
  // ...
  lcd.init();
  lcd.createChar(0, CharsConstants::MARKER);
  lcd.createChar(1, CharsConstants::STOP);
  lcd.createChar(2, CharsConstants::PUMP);
  lcd.createChar(3, CharsConstants::TEMP_OUT);
  lcd.createChar(4, CharsConstants::TEMP_IN);
  lcd.createChar(5, CharsConstants::EVP);
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.setCursor(0, 1);
  lcd.print("     TEPLOTECH.     ");
  lcd.setCursor(0, 2);
  lcd.print("     EVP-6 V1.0     ");
  lcd.setCursor(0, 3);
  delay(timeData.updateInterval);

  lcd.clear();
}
void loop()
{
  timeData.currentTime = millis();
  setupMenu();
  readTemperatures();
  byte keyPressed = readKey();
  temperatureData.t_wo_output = interpolate(temperatureData.outsideTemp, DataConstants::numPoints, temperatureData.t_n, temperatureData.t_wo);
  temperatureData.t_wi_output = interpolate(temperatureData.outsideTemp, DataConstants::numPoints, temperatureData.t_n, temperatureData.t_wi);
  if (LCDUpdater.isReady())
  {
    lcd.clear();
  }
  if (ThermReadWindow.isReady())
  {
    heaterController();
  }
  switch (systemState.menuState)
  {
  case MenuState::NORMAL:
    if (keyPressed == MenuConstants::OK)
    {
      systemState.menuState = MenuState::MAIN_MENU;
      lcd.clear();
      displayMenu();
    }
    else if (keyPressed == MenuConstants::LEFT)
    {
      systemState.stopState = (systemState.stopState == 1) ? HeaterState::OFF : HeaterState::ON;
      eepromData.stopState = systemState.stopState;
      eepromData.writeToEEPROM(DataConstants::startAddress);
    }
    displayNormalData();
    handleTemp(keyPressed);
    break;
  case MenuState::MAIN_MENU:
    handleMenu(keyPressed);
    displayMenu();
    break;
  case MenuState::SUB_MENU:
    if (systemState.subMenuState == NETWORK_SETTINGS)
    {
      handleNetworkSettingsMenu(keyPressed, systemState.settings);
    }
    else
    {
      handleSubMenu(keyPressed);
    }
    break;
  case MenuState::CH_HEATERS_STATUS:
    handleHeaterSettings();
    break;

  case MenuState::CH_PUMP_STATUS:
    handlePumpSettings();
    break;

  case MenuState::CH_WORK_MODE:
    handleModeSettings();
    break;

  default:
    // Обработка других состояний или ошибок
    break;
  }
}
void readTemperatures()
{
  float tempValues[DataConstants::NUM_TEMP_SENSORS];
  for (int i = 0; i < DataConstants::NUM_TEMP_SENSORS; ++i)
  {
    therm.setPin(PinConstants::TEMP_PINS[i]);
    tempValues[i] = therm.getTemp();
  }
  temperatureData.insideTemp = tempValues[0];
  temperatureData.outsideTemp = tempValues[1];
  temperatureData.waterInTemp = tempValues[2];
  temperatureData.waterOutTemp = tempValues[3];
}

void displayNormalData()
{
  printHeaterStatus();
  displaySetWoinTemp();
  displaySystemState();
  displayStatus(char(4), temperatureData.insideTemp, "E", 0, 1);
  displayWorkModeState();
  displayStatus(char(5), temperatureData.waterOutTemp, "E", 15, 1);
  displayStatus(char(3), temperatureData.outsideTemp, "E", 0, 2);
  displayPumpState();
  displayStatus(char(126), temperatureData.waterInTemp, "E", 15, 2);
  displayHSVal();
}

void displayStatus(char symbol, float value, const String &error, int x, int y)
{
  lcd.setCursor(x, y);
  lcd.print(symbol);
  lcd.print(":");
  lcd.print(int(value) < -50 ? error : String(int(value)));
}

void printHeaterStatus()
{
  lcd.setCursor(0, 0);

  if (systemState.stopState)
  {
    lcd.print("XXX");
    turnOffAllHeaters();
  }
  else
  {
    for (int i = 0; i < 3; i++)
    {
      char heaterChar = (systemState.heater[i] == 1) ? 'I' : 'O';
      lcd.print(heaterChar);
      setHeaterPin(i, systemState.heater[i]);
    }
  }
}

void turnOffAllHeaters()
{
  for (int i = 0; i < 3; i++)
  {
    digitalWrite(PinConstants::TEN_PINS[i], HIGH);
  }
}

void setHeaterPin(int heaterIndex, int state)
{
  digitalWrite(PinConstants::TEN_PINS[heaterIndex], (state == 1) ? LOW : HIGH);
}

void displaySetWoinTemp()
{
  lcd.setCursor(8, 0);
  lcd.print("[");
  lcd.print(static_cast<int>(systemState.setWoInTemp));
  lcd.print("]");
}

void displaySystemState()
{
  lcd.setCursor(17, 0);
  lcd.print(char(5));
  lcd.print(":");
  char stateSymbol = systemState.stopState == 1 ? 'S' : 'R';
  lcd.print(stateSymbol);

  for (int i = 0; i < 3; i++)
  {
    digitalWrite(PinConstants::TEN_PINS[i], systemState.heater[i] == 1 && !systemState.stopState ? LOW : HIGH);
  }
}

void displayPumpState()
{
  lcd.setCursor(8, 2);
  lcd.print(char(2));
  lcd.print(":");
  lcd.print(systemState.pumpState == 0 ? "off" : "on");
}

void displayWorkModeState()
{
  lcd.setCursor(8, 1);
  lcd.print(systemState.workMode == 0 ? "AUTO" : "MANU");
}

void displayHSVal()
{
  if (systemState.workMode == 0)
  {
    lcd.setCursor(0, 3);
    lcd.print(char(126));
    lcd.print(char(5));
    lcd.print("|");
    lcd.print(int(temperatureData.t_wi_output));
    lcd.print("|");
    lcd.setCursor(14, 3);
    lcd.print("|");
    lcd.print(int(temperatureData.t_wo_output));
    lcd.print("|");
    lcd.print(char(5));
    lcd.print(char(126));
  }
}

void displayMenu()
{
  int totalItems = sizeof(menuItems) / sizeof(menuItems[0]);       // Получаем общее количество пунктов меню
  int startIdx = systemState.currentMenuItem;                      // Индекс первого пункта для отображения
  int endIdx = startIdx + DataConstants::MAX_MENU_ITEMS_DISPLAYED; // Индекс последнего пункта для отображения

  if (endIdx >= totalItems)
  {
    endIdx = totalItems;                                         // Установка конечного индекса на общее количество пунктов
    startIdx = endIdx - DataConstants::MAX_MENU_ITEMS_DISPLAYED; // Пересчитываем начальный индекс, чтобы показать 4 пункта
    if (startIdx < 0)
    {
      startIdx = 0; // Обработка случая, когда менее 4 пунктов
    }
  }

  for (int i = startIdx; i < endIdx; i++)
  {
    lcd.setCursor(0, i - startIdx); // Установка позиции на экране LCD для пунктов меню
    lcd.print((i == systemState.currentMenuItem) ? char(0) : char(32));
    lcd.print(menuItems[i].label);
  }
}

void displaySubMenu()
{
  int totalSubMenuItems = 0;
  SubMenu *currentSubMenu = nullptr;

  if (systemState.subMenuState == SYSTEM_SETTINGS)
  {
    totalSubMenuItems = sizeof(subMenuSettings) / sizeof(subMenuSettings[0]);
    currentSubMenu = subMenuSettings;
  }
  else if (systemState.subMenuState == NETWORK_SETTINGS)
  {
    totalSubMenuItems = sizeof(subMenuNetworkSettings) / sizeof(subMenuNetworkSettings[0]);
    currentSubMenu = subMenuNetworkSettings;
  }

  for (int i = 0; i < totalSubMenuItems; i++)
  {
    lcd.setCursor(0, i); // Установка позиции на экране LCD для пунктов субменю
    lcd.print((i == systemState.currentMenuItem) ? char(0) : char(32));
    lcd.print(currentSubMenu[i].label);
  }
}

void handleMenu(byte keyPressed)
{
  MenuState previousMenuState = systemState.menuState;
  int totalItems = sizeof(menuItems) / sizeof(menuItems[0]);

  switch (keyPressed)
  {
  case MenuConstants::DOWN:
    systemState.currentMenuItem = (systemState.currentMenuItem - 1 + totalItems) % totalItems;
    break;

  case MenuConstants::UP:
    systemState.currentMenuItem = (systemState.currentMenuItem + 1) % totalItems;
    break;

  case MenuConstants::OK:
    handleMenuOK();
    break;

  case MenuConstants::LEFT:
    handleMenuLeft();
    break;
  }

  if (systemState.menuState != previousMenuState)
  {
    lcd.clear();
    displayMenu();
  }
}

void handleSubMenu(byte keyPressed)
{
  int totalItems = 0;
  SubMenu *currentSubMenu = nullptr;

  if (systemState.subMenuState == SYSTEM_SETTINGS)
  {
    totalItems = sizeof(subMenuSettings) / sizeof(subMenuSettings[0]);
    currentSubMenu = subMenuSettings;
  }
  else if (systemState.subMenuState == NETWORK_SETTINGS)
  {
    totalItems = sizeof(subMenuNetworkSettings) / sizeof(subMenuNetworkSettings[0]);
    currentSubMenu = subMenuNetworkSettings;
  }

  switch (keyPressed)
  {
  case MenuConstants::DOWN:
    systemState.currentMenuItem = (systemState.currentMenuItem - 1 + totalItems) % totalItems;
    break;

  case MenuConstants::UP:
    systemState.currentMenuItem = (systemState.currentMenuItem + 1) % totalItems;
    break;

  case MenuConstants::OK:
    handleSubMenuOK(currentSubMenu);
    break;

  case MenuConstants::LEFT:
    handleSubMenuLeft();
    break;
  }
  displaySubMenu();
}

// Обработка события OK в основном меню
void handleMenuOK()
{
  MenuItem *currentItem = &menuItems[systemState.currentMenuItem];
  if (currentItem->subMenu != nullptr)
  {
    systemState.menuState = SUB_MENU;
    systemState.subMenu = currentItem->subMenu;
    if (currentItem->action != nullptr)
    {
      currentItem->action();
    }
    systemState.currentMenuItem = 0;
    lcd.clear();
    displaySubMenu();
  }
  else if (currentItem->action != nullptr)
  {
    currentItem->action();
  }
}

// Обработка события LEFT в основном меню
void handleMenuLeft()
{
  eepromData.updateFromSystemState(systemState);
  eepromData.writeToEEPROM(DataConstants::startAddress);
  systemState.menuState = NORMAL;
  lcd.clear();
}

void handleSubMenuOK(SubMenu *currentSubMenu)
{
  if (currentSubMenu != nullptr)
  {
    SubMenu *currentItem = &currentSubMenu[systemState.currentMenuItem];
    if (currentItem->subMenu != nullptr)
    {
      systemState.previousSubMenu = systemState.subMenu;
      systemState.subMenu = currentItem->subMenu;
      if (currentItem->action != nullptr)
      {
        currentItem->action();
      }
    }
    else if (currentItem->action != nullptr)
    {
      currentItem->action();
    }

    systemState.currentMenuItem = 0;
    lcd.clear();
    displaySubMenu();
  }
}

void handleSubMenuLeft()
{
  if (systemState.menuState == SUB_MENU)
  {
    systemState.menuState = MAIN_MENU;
    systemState.currentMenuItem = 0;
    lcd.clear();
    displayMenu();
  }
}

void handleTemp(byte keyPressed)
{
  static int Temperature = systemState.setWoInTemp; // Используем статическую переменную для сохранения значения между вызовами функции

  if (keyPressed == MenuConstants::DOWN)
  {
    if (Temperature < 85)
    { // Убедимся, что значение не превышает 85
      Temperature++;
    }
  }
  else if (keyPressed == MenuConstants::UP)
  {
    if (Temperature > 0)
    { // Убедимся, что значение не меньше 0
      Temperature--;
    }
  }
  systemState.setWoInTemp = Temperature; // Присваиваем новое значение переменной systemState.setWoInTemp
  eepromData.setWoInTemp = systemState.setWoInTemp;
  eepromData.writeToEEPROM(DataConstants::startAddress);
}

void handleHeaterSettings()
{
  lcd.clear();
  int heaterValues[] = {systemState.heater[0], systemState.heater[1], systemState.heater[2]};
  const char *heaterNames[] = {"Heater A: ", "Heater B: ", "Heater C: "};

  int currentField = 0;

  while (true)
  {
    if (timeData.currentTime - timeData.lastActionTime >= timeData.updateInterval)
    {
      lcd.clear();
      timeData.lastActionTime = timeData.currentTime;
    }
    for (int i = 0; i < 3; i++)
    {
      systemState.heater[i] = heaterValues[i];
      digitalWrite(PinConstants::TEN_PINS[i], heaterValues[i] ? LOW : HIGH);
    }
    lcd.setCursor(0, 0);
    lcd.print("Heaters On/Off");
    for (int i = 0; i < 3; i++)
    {
      lcd.setCursor(0, i + 1);
      lcd.print(i == currentField ? char(0) : char(32));
      lcd.print(i + 1);
      lcd.print(": ");
      lcd.print(heaterNames[i]);
      lcd.print(heaterValues[i] == 1 ? "on" : "off");
      lcd.print("   ");
    }

    byte keyPressed = readKey();

    if (keyPressed == MenuConstants::DOWN)
    {
      currentField = (currentField + 1) % 3;
    }
    else if (keyPressed == MenuConstants::UP)
    {
      currentField = (currentField + 1) % 3;
    }
    else if (keyPressed == MenuConstants::RIGHT || keyPressed == MenuConstants::LEFT)
    {
      heaterValues[currentField] = !heaterValues[currentField]; // Toggle heater value
      digitalWrite(PinConstants::TEN_PINS[currentField], heaterValues[currentField] ? LOW : HIGH);
    }
    else if (keyPressed == MenuConstants::OK)
    {
      for (int i = 0; i < 3; i++)
      {
        systemState.heater[i] = heaterValues[i];
        digitalWrite(PinConstants::TEN_PINS[i], heaterValues[i] ? LOW : HIGH);
      }
      systemState.menuState = MAIN_MENU;
      break;
    }
  }
}

void handlePumpSettings()
{
  lcd.clear();
  const char *pumpName = "Pump: ";
  while (true)
  {
    if (timeData.currentTime - timeData.lastActionTime >= timeData.updateInterval)
    {
      lcd.clear();
      timeData.lastActionTime = timeData.currentTime;
    }
    controlPump();
    lcd.setCursor(0, 0);
    lcd.print("Pump On/Off");
    lcd.setCursor(0, 1);
    lcd.print(char(0));
    // lcd.print(1);
    // lcd.print(": ");
    lcd.print(pumpName);
    lcd.print(systemState.pumpState == 1 ? "on" : "off");
    lcd.print("   ");

    byte keyPressed = readKey();

    if (keyPressed == MenuConstants::RIGHT || keyPressed == MenuConstants::LEFT)
    {
      systemState.pumpState = !systemState.pumpState; // Toggle pump value
      controlPump();
    }
    else if (keyPressed == MenuConstants::OK)
    {
      systemState.menuState = MAIN_MENU;
      break;
    }
  }
}

void handleModeSettings()
{
  lcd.clear();
  const char *ItemName = "Mode: ";

  while (true)
  {
    if (timeData.currentTime - timeData.lastActionTime >= timeData.updateInterval)
    {
      lcd.clear();
      timeData.lastActionTime = timeData.currentTime;
    }
    lcd.setCursor(0, 0);
    lcd.print("Work mode");
    lcd.setCursor(0, 1);
    lcd.print(char(0));

    lcd.print(ItemName);
    lcd.print(systemState.workMode == 0 ? "AUTO" : "MANU");

    byte keyPressed = readKey();

    if (keyPressed == MenuConstants::RIGHT || keyPressed == MenuConstants::LEFT)
    {
      systemState.workMode = !systemState.workMode; // Toggle pump value
      // lcd.print(systemState.workMode == 0 ? "AUTO" : "MANU");
    }
    else if (keyPressed == MenuConstants::OK)
    {
      systemState.menuState = MAIN_MENU;
      break;
    }
  }
}

void handleNetworkSettingsMenu(byte keyPressed, NetworkSettings &settings)
{
  int selectedItem = systemState.selectedItem;
  int mode = systemState.NetworkEditMode;
  int tempItem = systemState.tempItem;
  int totalItems = 3;
  switch (keyPressed)
  {
  case MenuConstants::UP:
    if (mode == 0)
    {
    }
    else if (mode == 1)
    {
      if (settings.ipAddress[tempItem] < '9')
      {
        settings.ipAddress[tempItem]++;
      }
      else
      {
        settings.ipAddress[tempItem] = '0';
      }
    }
    break;
  case MenuConstants::DOWN:
    if (mode == 0)
    {
      if (settings.ipAddress[tempItem] > '0')
      {
        settings.ipAddress[tempItem]--;
      }
      else
      {
        settings.ipAddress[tempItem] = '9';
      }
    }
    else if (mode == 1)
    {
      if (settings.ipAddress[tempItem] > '0')
      {
        settings.ipAddress[tempItem]--;
      }
      else
      {
        settings.ipAddress[tempItem] = '9';
      }
    }
    break;
  case MenuConstants::LEFT:
    if (mode == 1)
    {
      if (selectedItem > 0)
      {
        selectedItem--;
      }
    }
    else if (mode == 3)
    {
      if (selectedItem > 0)
      {
        selectedItem--;
      }
    }
    break;
  case MenuConstants::RIGHT:
    if (mode == 1)
    {
      if (selectedItem < 14)
      {
        selectedItem++;
      }
    }
    else if (mode == 3)
    {
      if (selectedItem < 14)
      {
        selectedItem++;
      }
    }
    break;
  case MenuConstants::OK:
    if (mode == 0 && selectedItem == 0 || 1 || 2)
    {
    }
    else if (mode == 1 && selectedItem == 0 || 1 || 2)
    {
    }
    else if (selectedItem == 3)
    {
    }
    break;
  }
}

void heaterController()
{
  float setWoInTemp = systemState.setWoInTemp;
  if (systemState.workMode == 0)
  {
    if (timeData.currentTime - timeData.lastTempActionTime >= timeData.updateInterval)
    {
      timeData.lastTempActionTime = timeData.currentTime;
      if (temperatureData.waterOutTemp < temperatureData.t_wo_output)
      {
        int heatersToTurnOn = 1;
        for (int i = 0; i < DataConstants::MAX_HEATERS && heatersToTurnOn > 0; ++i)
        {
          if (systemState.heater[i] == 0)
          {
            systemState.heater[i] = 1;
            heatersToTurnOn--;
          }
        }
        if (heatersToTurnOn == 0)
        {
          systemState.tempReached = 0;
        }
      }
      else if (temperatureData.waterOutTemp >= temperatureData.t_wo_output)
      {
        for (int i = 0; i < DataConstants::MAX_HEATERS; ++i)
        {
          systemState.heater[i] = 0;
        }
        systemState.tempReached = 1;
      }
    }
  }
  else if (systemState.workMode == 1)
  {
    if (timeData.currentTime - timeData.lastTempActionTime >= timeData.updateInterval)
    {
      timeData.lastTempActionTime = timeData.currentTime;
      if (temperatureData.waterOutTemp < setWoInTemp)
      {
        for (int i = 0; i < DataConstants::MAX_HEATERS; ++i)
        {
          systemState.heater[i] = systemState.tempHeater[i];
        }
        systemState.tempReached = 0;
      }
      else if (temperatureData.waterOutTemp >= setWoInTemp)
      {
        for (int i = 0; i < DataConstants::MAX_HEATERS; ++i)
        {
          systemState.tempHeater[i] = systemState.heater[i];
          systemState.heater[i] = 0;
        }
        systemState.tempReached = 1;
      }
    }
  }
}

void controlPump()
{
  digitalWrite(PinConstants::PUMP_PIN, systemState.pumpState ? LOW : HIGH);
}