/*
 * Скетч управления системой полива "Огород"
 * Версия: 2.1
 * Оборудование: ESP32, LCD1602, TCA9555, датчики
 *
 * Изменения v2.1:
 * - Ультразвуковой датчик читается раз в 1 сек (не каждый loop)
 * - DHT11 читается раз в 2 сек
 * - I2C recovery не чаще раза в 5 сек (защита от рекурсии)
 * - MQTT reconnect при потере связи с брокером
 * - readSensors() вынесена в таймер
 */

#include <DHT.h>
#include <Wire.h>
#define _LCD_TYPE 1
//#include <LiquidCrystal_I2C.h>
#include <I2C_LiquidCrystal_RUS.h>
#include <GyverNTP.h>
#include <RCSwitch.h>
#include <Arduino.h>
#include <FlowSensor.h>
#include <GyverHub.h>
#include <FastBot2.h>
#include <PairsFile.h>
#include <GyverDS18.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Preferences.h>   // NVS для сохранения режимов и настроек
#include "esp_task_wdt.h"  // Watchdog для автоматического сброса при зависании

// =================== ЗАЩИТА I2C ОТ ПРЕРЫВАНИЙ ===================
// FlowSensor (YF-S201) генерирует прерывания до 450 Гц,
// которые сбивают I2C транзакции. Защита:
// - ошибка I2C → recovery (не чаще 1 раза в 5 сек)
// - профилактика раз в 10 мин
unsigned long lastI2cRecovery = 0;
const unsigned long I2C_RECOVERY_INTERVAL = 600000; // 10 минут — профилактика

bool i2cError = false;                    // Флаг ошибки I2C
uint16_t relayState = 0xFFFF;             // Состояние реле TCA9555

// Временное отключение прерываний FlowSensor перед I2C операциями
// Используем gpio_intr_disable (быстрее detachInterrupt, не теряем настройку)
// Номера пинов: manPin=27 (полив), fillPin=14 (налив) — см. defines ниже
void disableFlowIRQ() {
  gpio_intr_disable(GPIO_NUM_27);
  gpio_intr_disable(GPIO_NUM_14);
}
void enableFlowIRQ() {
  gpio_intr_enable(GPIO_NUM_27);
  gpio_intr_enable(GPIO_NUM_14);
}

// Функции i2cBusRecovery и lcd-обёртки определены ниже, после объявления lcd

// =================== КОНФИГУРАЦИЯ ===================
#define AP_SSID "MERCUSYS_3BCA"
#define AP_PASS "router320"
//#define BOT_TOKEN "7358886965:AAH2yQT5gs6qFp4XQHRzkrsbsoLGZlS-ItU"
//#define CHAT_ID "5286033464"
#define DHTTYPE DHT11       // Тип датчика DHT

// =================== ПИНЫ ESP32 ===================
#define DHTPIN 17           // Датчик температуры и влажности DHT11
#define PIN_TRIG 5          // Ультразвуковой датчик уровня (TRIG)
#define PIN_ECHO 18         // Ультразвуковой датчик уровня (ECHO)
#define manPin 27           // Датчик потока полив YF-S201
#define fillPin 14          // Датчик потока налив YF-S201 (освобожден ZAKR)
#define soilTempPin 32      // Датчик температуры почвы DS18B20 (освобожден UPR_1)
#define btnModePin 33       // Кнопка выбора режима (освобожден UPR_2)
#define btnConfirmPin 25    // Кнопка подтверждения (освобожден UPR_3)
#define gruntPin 4          // ADC датчик влажности почвы (огород)
#define gruntGreenhousePin 34 // ADC датчик влажности почвы (теплица)
#define krushPin 19         // Свободен (был цифровой датчик влажности)
#define rcPin 16            // RF передатчик
#define relayPhylampPin 26  // Реле фитолампы на плате ESP32
#define relayFillPin 2      // Реле клапана налива (внешнее реле или на плате)

#define TCA9555_ADDR 0x20   // I2C адрес TCA9555 (A0-A3 на GND)
// Выходы TCA9555 (P00-P07) -> 8-канальный релейный модуль
#define TCA_RELAY_PUMP      0   // P00 - насос
#define TCA_RELAY_GREENHOUSE 1  // P01 - клапан теплицы
#define TCA_RELAY_DRIP      2   // P02 - клапан капельного полива
#define TCA_RELAY_DIR1      3   // P03 - клапан 1 направления
#define TCA_RELAY_DIR2      4   // P04 - клапан 2 направления
#define TCA_RELAY_DIR3      5   // P05 - клапан 3 направления
#define TCA_RELAY_PHASE     6   // P06 - подача фазы 10 сек
#define TCA_RELAY_FRAMUGA   7   // P07 - фрамуга

// Входы TCA9555 (P10-P14) -> контроль запорной арматуры
#define TCA_INPUT_MAIN_VALVE    8   // P10 - главный кран (ввод с бака)
#define TCA_INPUT_GREENHOUSE    9   // P11 - состояние клапана теплицы
#define TCA_INPUT_DRIP          10  // P12 - состояние клапана капельного
#define TCA_INPUT_DIR1          11  // P13 - состояние клапана 1 направления
#define TCA_INPUT_DIR2          12  // P14 - состояние клапана 2 направления

GyverHub hub("MyDevices", "Ogorod_esp32", "");
PairsFile data(&GH_FS, "/data.dat", 3000);
File historyFile;  // Файл для хранения исторических данных
const char* HISTORY_FILE = "/history.csv";  // CSV файл с историей
DHT dht(DHTPIN, DHTTYPE);
I2C_LiquidCrystal_RUS lcd(0x27, 16, 2);
FlowSensor SensorPoliv(YFS201, manPin);
FlowSensor SensorNaliv(YFS201, fillPin);
RCSwitch mySwitch = RCSwitch();
GyverNTP ntp(3);
GyverDS18 soilSensor(soilTempPin);  // Датчик температуры почвы DS18B20

// =================== ЗАЩИТА I2C ОТ ПРЕРЫВАНИЙ ===================
// Обёртки LCD с отключением прерываний FlowSensor на время I2C транзакции.
// FlowSensor (YF-S201) генерирует прерывания до 450 Гц, которые сбивают
// I2C транзакции LCD. disableFlowIRQ отключает только 2 GPIO прерывания
// (не все, как noInterrupts), поэтому блокировка до ~10 мс безопасна
// для FreeRTOS.
void lcdClear()        { disableFlowIRQ(); lcd.clear(); enableFlowIRQ(); }
void lcdBacklight()    { disableFlowIRQ(); lcd.backlight(); enableFlowIRQ(); }
void lcdPrint(const char* s)  { disableFlowIRQ(); lcd.print(s); enableFlowIRQ(); }
void lcdPrint(const String& s){ disableFlowIRQ(); lcd.print(s); enableFlowIRQ(); }
void lcdSetCursor(int c, int r) { disableFlowIRQ(); lcd.setCursor(c, r); enableFlowIRQ(); }
void lcdInit() {
  disableFlowIRQ();
  lcd.init();
  enableFlowIRQ();
  delay(50);  // LCD1602 требует паузу после инициализации
}

void i2cBusRecovery() {
  Serial.println("I2C recovery started");

  // Отключаем прерывания FlowSensor на время переинициализации I2C
  disableFlowIRQ();

  // 1. Hardware recovery — тактируем SCL (8-10 импульсов) чтобы освободить устройства
  pinMode(22, OUTPUT);  // SCL
  pinMode(21, OUTPUT);  // SDA
  digitalWrite(21, HIGH); // SDA = HIGH
  for (int i = 0; i < 10; i++) {
    digitalWrite(22, LOW);
    delayMicroseconds(10);
    digitalWrite(22, HIGH);
    delayMicroseconds(10);
  }

  // 2. Переинициализация Wire
  Wire.end();
  delay(10);
  Wire.begin();
  Wire.setTimeOut(100);

  // 3. Переинициализация LCD
  lcdInit();
  lcdBacklight();
  lcdClear();

  // 4. Переинициализация TCA9555 с восстановлением состояния реле
  TCA9555_init();
  TCA9555_write(relayState);

  // 5. Верификация — пингуем TCA9555
  Wire.beginTransmission(TCA9555_ADDR);
  byte err = Wire.endTransmission();

  enableFlowIRQ();  // включаем прерывания FlowSensor обратно

  if (err == 0) {
    Serial.println("I2C recovery OK");
    lcdPrint("I2C OK");
  } else {
    Serial.print("I2C recovery FAIL: ");
    Serial.println(err);
    lcdPrint("I2C FAIL");
  }

  i2cError = false;
  lastI2cRecovery = millis();
}

// =================== ПЕРЕМЕННЫЕ ===================
long duration, cm;
static long cmFilt = -1;     // фильтрованное значение уровня
static int cmFailCnt = 0;    // ошибок подряд

float t, h, d_poliv, d_naliv;
float soilTemp = 0;
int g;                      // ADC влажность почвы огород
int g_greenhouse;           // ADC влажность почвы теплица
bool k = 0;                 // Влажность почвы огород (1=сухо, 0=мокро) — из analogRead(gruntPin)
bool k_greenhouse = 0;      // Влажность почвы теплица (1=сухо, 0=мокро) — из analogRead(gruntGreenhousePin)
int j = 0;                  // Выбор режима полива (0-5)
int y_greenhouse = 6 ;       // Часы старта - теплица
int y_drip = 7;             // Часы старта - капельный
int y_sprinkler = 8;        // Часы старта - спринклер
int y_phylamp = 18;         // Часы старта - фитолампа
int y_twice_week = 9;       // Часы старта - 2 раза в неделю
int y_weekly = 10;          // Часы старта - еженедельно
int duration_greenhouse = 30;   // Длительность полива теплицы (мин)
int duration_drip = 10;         // Длительность полива капельный (мин)
int duration_sprinkler = 15;    // Длительность полива спринклер (мин)

String stamp_1, stamp_2;
String val, val_1, val_2, val_3;
const String s_1("дождь");
const String s_2("сухая");
const String s_3("высохла");
const String s_4("влажная");
const String s_5("мокрая");
const String s_6("насос вкл");
const String s_7("клапан открыт");
const String s_8("выключено");
static byte tab;
static byte sel;
int o = 0;  // счетчик для телеграмма
bool mode_greenhouse = false;   // Режим теплицы
bool mode_drip = false;         // Режим капельный
bool mode_sprinkler = false;    // Режим спринклерный
bool mode_fill = false;         // Режим автоналива
bool mode_phylamp = false;      // Режим фитолампы
bool mode_framuga = false;      // Режим фрамуги
bool mode_twice_week = false;   // Режим 2 раза в неделю
bool mode_weekly = false;       // Режим еженедельно

// Флаги активных процессов
bool flag_pump = false;
bool flag_greenhouse = false;
bool flag_drip = false;
bool flag_dir1 = false;
bool flag_dir2 = false;
bool flag_dir3 = false;
bool flag_phase = false;
bool flag_framuga = false;
bool flag_phylamp = false;
bool flag_fill = false;
bool flowAlarm = false;  // Авария потока — насос работает, а воды нет

// Состояние входов TCA9555 (запорная арматура)
bool inputMainValve = false;    // Главный кран (ввод с бака)
bool inputGreenhouse = false;   // Состояние клапана теплицы
bool inputDrip = false;         // Состояние клапана капельного
bool inputDir1 = false;         // Состояние клапана 1 направления
bool inputDir2 = false;         // Состояние клапана 2 направления

// Состояние кнопок
bool btnModePressed = false;
bool btnConfirmPressed = false;
unsigned long btnModePressTime = 0;
unsigned long btnConfirmPressTime = 0;
const unsigned long LONG_PRESS_TIME = 1500;  // 1.5 сек для удержания

// Таймеры процессов
unsigned long pumpStartTime = 0;
unsigned long phaseStartTime = 0;
unsigned long dirStartTime = 0;
unsigned long flowCheckTime = 0;
bool flowDetected = false;

// =================== ТАЙМЕРЫ GH::TIMER ===================
 static gh::Timer tmr_process;            // Общий таймер процесса
static gh::Timer tmr_pump;               // Таймер насоса
static gh::Timer tmr_flow;               // Таймер проверки потока
static gh::Timer tmr_phase;              // Таймер фазы
static gh::Timer tmr_dir;                // Таймер направления
static gh::Timer tmr_phylamp_process;    // Таймер фитолампы
static gh::Timer tmr_lcd_timeout;        // Таймаут LCD меню

// Состояние LCD
enum LCDState {
  LCD_TIME,
  LCD_LEVEL,
  LCD_MODE_SELECT,
  LCD_ACTIVE_MODE
};
LCDState lcdState = LCD_TIME;
LCDState prevLcdState = LCD_TIME;  // Для отслеживания изменений
unsigned long lcdTimer = 0;
const unsigned long LCD_TIMEOUT = 10000;  // 10 сек возврата к времени
int prevJ = -1;  // Для отслеживания изменения режима в меню

bool historyWritten = false;  // Флаг записи данных за сегодня
int lastHistoryDay = 0;       // Последний день записи
String graphData = "";        // Данные для графика
bool graphRequested = false;  // Флаг запроса графика

// Показ IP на LCD при старте
unsigned long showIpUntil = 0;
String ipAddr = "";

// Переменные для прогноза погоды (ОТКЛЮЧЕНО)
 String weatherForecast = "Нет данных";  // Прогноз погоды на неделю
 bool weatherRequested = false;  // Флаг запроса погоды
 const String YANDEX_API_KEY = "40bb878f-870b-4f56-a2aa-fd6173e8df25";  // Замените на ваш API ключ Яндекс.Погода
 const String WEATHER_LAT = "47.224861";  // 47°13′29.50″ с.ш.
 const String WEATHER_LON = "39.702286";  // 39°42′08.23″ в.д.

 gh:: Button btn_show_graph;
 gh:: Button btn_clear_graph;
 gh:: Button btn_weather_forecast;  // ОТКЛЮЧЕНО

 // =================== TCA9555 ФУНКЦИИ ===================
void TCA9555_init() {
  disableFlowIRQ();
  Wire.beginTransmission(TCA9555_ADDR);
  Wire.write(0x06);  // Configuration register 0
  Wire.write(0x00);  // Порт 0: все пины на выход (реле)
  Wire.write(0xFF);  // Configuration register 1: Порт 1: все пины на вход (контроль клапанов)
  byte err = Wire.endTransmission();
  enableFlowIRQ();
  if (err != 0) i2cError = true;
}

void TCA9555_write(uint16_t value) {
  disableFlowIRQ();
  Wire.beginTransmission(TCA9555_ADDR);
  Wire.write(0x02);  // Output register 0
  Wire.write(value & 0xFF);       // Порт 0 (реле)
  Wire.write((value >> 8) & 0xFF); // Порт 1 (не используется для записи)
  byte err = Wire.endTransmission();
  enableFlowIRQ();
  if (err != 0) i2cError = true;
}

void TCA9555_setRelay(byte relay, bool state) {
  if (state) {
    relayState &= ~(1 << relay);  // Включаем реле (устанавливаем LOW)
  } else {
    relayState |= (1 << relay);   // Выключаем реле (устанавливаем HIGH)
  }
  TCA9555_write(relayState);
}

void TCA9555_allOff() {
  TCA9555_write(0x00FF);  // Все выходы в HIGH (реле выключены, активный LOW)
}

// Чтение состояния входов TCA9555 (порт 1)
void TCA9555_readInputs() {
  disableFlowIRQ();
  Wire.beginTransmission(TCA9555_ADDR);
  Wire.write(0x01);  // Input register 1
  if (Wire.endTransmission() != 0) { enableFlowIRQ(); i2cError = true; return; }

  if (Wire.requestFrom(TCA9555_ADDR, 1) != 1) { enableFlowIRQ(); i2cError = true; return; }
  if (Wire.available()) {
    byte inputState = Wire.read();
    enableFlowIRQ();
    inputMainValve = bitRead(inputState, 0);  // P10
    inputGreenhouse = bitRead(inputState, 1); // P11
    inputDrip = bitRead(inputState, 2);       // P12
    inputDir1 = bitRead(inputState, 3);       // P13
    inputDir2 = bitRead(inputState, 4);       // P14
  } else {
    enableFlowIRQ();
  }
}
// =================== ОБРАБОТЧИКИ КНОПОК ===================
void IRAM_ATTR btnModeISR() {
  btnModePressed = true;
}

void IRAM_ATTR btnConfirmISR() {
  btnConfirmPressed = true;
}

void checkButtons() {
  // Кнопка выбора режима
  if (digitalRead(btnModePin) == LOW) {
    if (btnModePressTime == 0) {
      btnModePressTime = millis();
    } else if (millis() - btnModePressTime >= LONG_PRESS_TIME) {
       //Долгое нажатие - вход/выход из меню выбора режимов
      if (lcdState != LCD_MODE_SELECT) {
        lcdState = LCD_MODE_SELECT;
        tmr_lcd_timeout.setTime(LCD_TIMEOUT);
        tmr_lcd_timeout.startTimeout();
        prevJ = -1;  // Сброс для гарантированной перерисовки
      } else {
        lcdState = LCD_TIME;  // Повторное долгое нажатие — возврат к времени
        lcdClear();
        prevLcdState = LCD_TIME;
      }
      btnModePressTime = millis();  // Сброс для следующего нажатия
    }
  } else {
    if (btnModePressTime > 0 && millis() - btnModePressTime < LONG_PRESS_TIME) {
      // Короткое нажатие - переключение режима
      if (lcdState == LCD_MODE_SELECT) {
        j = (j + 1) % 6;  // 6 режимов (0-5)
        tmr_lcd_timeout.setTime(LCD_TIMEOUT);
        tmr_lcd_timeout.startTimeout();
      }
    }
   btnModePressTime = 0;
  }

  // Кнопка подтверждения
  if (digitalRead(btnConfirmPin) == LOW) {
    if (btnConfirmPressTime == 0) {
      btnConfirmPressTime = millis();
    } else if (millis() - btnConfirmPressTime >= LONG_PRESS_TIME) {
      // Долгое нажатие - показать уровень бака
      if (lcdState != LCD_LEVEL) {
        lcdState = LCD_LEVEL;
        lcdClear();
        prevJ = -1;
        tmr_lcd_timeout.setTime(LCD_TIMEOUT);
        tmr_lcd_timeout.startTimeout();
      }
      btnConfirmPressTime = millis();
    }
  } else {
    if (btnConfirmPressTime > 0 && millis() - btnConfirmPressTime < LONG_PRESS_TIME) {
      // Короткое нажатие - подтверждение выбора режима
      if (lcdState == LCD_MODE_SELECT) {
        // Подтверждение выбора
        // Включаем/выключаем выбранный режим
        switch (j) {
          case 0: mode_greenhouse = !mode_greenhouse; break;
          case 1: mode_drip = !mode_drip; break;
          case 2: mode_sprinkler = !mode_sprinkler; break;
          case 3: mode_fill = !mode_fill; break;
          case 4: mode_phylamp = !mode_phylamp; break;
          case 5: mode_framuga = !mode_framuga; break;
        }
        hub.sendRefresh();
        // Сбрасываем prevJ для гарантированной перерисовки экрана с новым состоянием on/off
        prevJ = -1;
        // Возврат к времени с задержкой для показа состояния
        tmr_lcd_timeout.setTime(LCD_TIMEOUT);
        tmr_lcd_timeout.startTimeout();
      }
    }
    btnConfirmPressTime = 0;
  }

  // Таймаут возврата к времени
  if (lcdState == LCD_MODE_SELECT && tmr_lcd_timeout) {
    lcdState = LCD_TIME;
  }
}

// =================== ОБРАБОТЧИКИ FLOW SENSOR ===================
IRAM_ATTR void countPoliv() {
  SensorPoliv.count();
}

IRAM_ATTR void countNaliv() {
  SensorNaliv.count();
}

// =================== ФУНКЦИИ ИСТОРИИ ДАННЫХ ===================

// Запись данных в файл истории (температура, влажность, дата)
void writeHistoryData() {
  int currentDay = ntp.day();

  // Записываем только если наступил новый день и данные еще не записаны
  if (currentDay != lastHistoryDay && !historyWritten) {
    // Сначала читаем существующие данные
    String existingData = "";
    if (GH_FS.exists(HISTORY_FILE)) {
      File readFile = GH_FS.open(HISTORY_FILE, "r");
      if (readFile) {
        existingData = readFile.readString();
        readFile.close();
      }
    }

    // Открываем файл для записи (перезапись с добавлением новых данных)
    historyFile = GH_FS.open(HISTORY_FILE, "w");

    if (historyFile) {
      // Записываем старые данные + новые
      historyFile.print(existingData);

      // Формируем строку CSV: день,температура,влажность
      String dataLine = String(currentDay) + "," +
                        String(t, 1) + "," +
                        String(h, 1) + "\n";
      historyFile.print(dataLine);
      historyFile.close();

      lastHistoryDay = currentDay;
      historyWritten = true;
      Serial.println("Данные записаны в историю: " + dataLine);
    } else {
      Serial.println("Ошибка открытия файла истории");
    }
  }
}

// Чтение данных из файла истории для графика
String readHistoryData() {
  String result = "";

  historyFile = GH_FS.open(HISTORY_FILE, "r");

  if (historyFile) {
    // Читаем файл построчно
    while (historyFile.available()) {
      String line = historyFile.readStringUntil('\n');
      if (line.length() > 0) {
        // Форматируем для GyverHub Graph
        // Формат: день:температура:влажность
        result += line + "|";
      }
    }
    historyFile.close();
  } else {
    Serial.println("Файл истории не найден");
    return "Нет данных";
  }

  return result;
}

// Очистка файла истории (по кнопке)
void clearHistoryData() {
  if (GH_FS.exists(HISTORY_FILE)) {
    GH_FS.remove(HISTORY_FILE);
    Serial.println("История очищена");
  }
}



// Запрос прогноза погоды на неделю через Яндекс.Погода API (ОТКЛЮЧЕНО)
 void requestWeatherForecast() {
   if (YANDEX_API_KEY == "ВАШ_API_КЛЮЧ_YANDEX") {
     weatherForecast = "Укажите API ключ Яндекс.Погода";
     Serial.println("Не указан API ключ для погоды");
     return;
   }
//
//   // Формируем URL запроса к Яндекс.Погоде
//   // Используем эндпоинт /v2/informers/telemetry для прогноза
   String url = "https://api.weather.yandex.ru/v2/forecast?lat=" + WEATHER_LAT +
                "&lon=" + WEATHER_LON +
               "&limit=7" +
                "&lang=ru_RU";

   HTTPClient http;
   http.begin(url);
   http.setTimeout(10000);
//
//   // Добавляем заголовок с API ключом Яндекса
   http.addHeader("X-Yandex-API-Key", YANDEX_API_KEY);
//
   int httpCode = http.GET();
//
   if (httpCode == HTTP_CODE_OK) {
     String payload = http.getString();
//
     // Парсим JSON
     StaticJsonDocument<8192> doc;
     DeserializationError error = deserializeJson(doc, payload);
//
     if (!error) {
//       // Формируем прогноз на 7 дней из Яндекс.Погоды
       weatherForecast = "Прогноз на 7 дней (Яндекс):\n";
//
//       // Яндекс возвращает массив forecasts с ежедневными данными
       JsonArray forecasts = doc["forecasts"];
//
       for (size_t i = 0; i < forecasts.size() && i < 7; i++) {
         JsonObject forecast = forecasts[i];
         String date = forecast["date"].as<String>();
//
//         // Дневной прогноз
         JsonObject daytime = forecast["parts"]["day_short"];
         if (daytime.isNull()) {
           daytime = forecast["parts"]["day"];
         }
//
         float temp = daytime["temp"].as<float>();
         float feelsLike = daytime["feels_like"].as<float>();
         int humidity = daytime["humidity"].as<int>();
         String condition = daytime["condition"].as<String>();
         float windSpeed = daytime["wind_speed"].as<float>();
//
//         // Преобразуем условие в понятный текст
         String conditionText = condition;
         if (condition == "clear") conditionText = "ясно";
         else if (condition == "partly-cloudy") conditionText = "малооблачно";
         else if (condition == "cloudy") conditionText = "облачно";
        else if (condition == "overcast") conditionText = "пасмурно";
        else if (condition == "drizzle") conditionText = "морось";
         else if (condition == "light-rain") conditionText = "небольшой дождь";
         else if (condition == "rain") conditionText = "дождь";
         else if (condition == "moderate-rain") conditionText = "умеренный дождь";
         else if (condition == "heavy-rain") conditionText = "сильный дождь";
         else if (condition == "continuous-heavy-rain") conditionText = "очень сильный дождь";
         else if (condition == "showers") conditionText = "ливень";
         else if (condition == "wet-snow") conditionText = "дождь со снегом";
         else if (condition == "snow") conditionText = "снег";
         else if (condition == "snow-showers") conditionText = "снегопад";
         else if (condition == "hail") conditionText = "град";
         else if (condition == "thunderstorm") conditionText = "гроза";
         else if (condition == "thunderstorm-with-rain") conditionText = "гроза с дождем";
         else if (condition == "thunderstorm-with-snow") conditionText = "гроза со снегом";
//
         weatherForecast += date + ": ";
         weatherForecast += "t=" + String(temp, 1) + "°C (ощущ." + String(feelsLike, 1) + "°C),\n ";
         weatherForecast += "вл=" + String(humidity) + "%, ";
         weatherForecast += conditionText + ", ";
         weatherForecast += "ветер=" + String(windSpeed, 1) + "м/с\n";
       }
//
       Serial.println("Прогноз погоды Яндекс получен успешно");
     } else {
       weatherForecast = "Ошибка парсинга JSON";
       Serial.println("Ошибка парсинга: " + String(error.c_str()));
     }
   } else {
     weatherForecast = "Ошибка HTTP: " + String(httpCode);
     Serial.println("Ошибка HTTP запроса к Яндекс.Погоде: " + String(httpCode));
     Serial.println("Ответ: " + http.getString());
   }
//
   http.end();
 }

void build(gh::Builder& b) {
  if (b.beginRow()) {
    b.Label("дата ").label(F("ВРЕМЯ")).size(2, 1).fontSize(25).value(stamp_1);
    b.Label("время ").label(F("ДАТА")).size(2, 1).fontSize(25).value(stamp_2);
    b.endRow();
  }

  {
    gh::Row r(b);
    b.Display_("disp_1", val).label(F("влажность земли на огороде")).size(3);
     b.Label_(F("disp_3")).label(F("значение влажности")).size(2).fontSize(20).value("0");
  }
  {
    gh::Row r(b);
    b.Display_("disp_2", val_1).label(F("влажность почвы в теплице")).size(3);
     b.Label_(F("disp_4")).label(F("значение влажности")).size(2).fontSize(20).value("0");
  }

  {
    gh::Row r(b);
    if (b.Tabs(&tab).label(F("РЕЖИМ")).text("стоп;пуск").click()) b.refresh();
    switch (tab) {
      case 0:
        Serial.println("стоп");
        break;
      case 1:
        Serial.println("пуск");
        break;
    }
  }

  if (b.beginRow()){
    b.Label_(F("tim_gh")).label(F("таймеры")).size(3).fontSize(16).value("настр");
    b.Input_("inp_gh", &y_greenhouse).label(F("теплица часы полив")).size(1).fontSize(10).value(y_greenhouse).color(gh::Colors::Blue);
    b.Input_("inp_gh2", &y_drip).label(F("капельный часы полива")).size(1).fontSize(10).value(y_drip).color(gh::Colors::Blue);
    b.Input_("inp_gh3", &y_sprinkler).label(F("спринклер часы полив")).size(1).fontSize(10).value(y_sprinkler).color(gh::Colors::Blue);
    b.endRow();
  }
  if (b.beginRow()){
    b.Input_("inp_gh4", &y_phylamp).label(F("фитолампа часы вкл")).size(2).fontSize(10).value(y_phylamp).color(gh::Colors::Blue);
    b.Input_("inp_gh5", &y_twice_week).label(F("2 раза в неделю часы работы")).size(2).fontSize(10).value(y_twice_week).color(gh::Colors::Blue);
    b.Input_("inp_gh6", &y_weekly).label(F("раз в неделю часы работы")).size(2).fontSize(10).value(y_weekly).color(gh::Colors::Blue);
    b.endRow();
  }

  {
    gh::Row r(b);
    b.Label_(F("pot_poliv")).label(F("поток полив")).size(2).fontSize(20).value(d_poliv, 2);
    b.Label_(F("pot_naliv")).label(F("поток налив")).size(2).fontSize(20).value(d_naliv, 2);
    b.LED_("ld_alarm_level").label(F("авар уровень")).color(gh::Colors::Red).value(0);
    b.LED_("ld_alarm_flow").label(F("авар поток")).color(gh::Colors::Red).value(0);
  }

  // Виджеты состояния режимов
  {
    gh::Row r(b);
    b.LED_("ld_greenhouse").label(F("теплица")).color(gh::Colors::Green).value(mode_greenhouse);
    b.LED_("ld_drip").label(F("капельный")).color(gh::Colors::Green).value(mode_drip);
    b.LED_("ld_sprinkler").label(F("спринклер")).color(gh::Colors::Green).value(mode_sprinkler);
  }

  {
    gh::Row r(b);
    b.LED_("ld_fill").label(F("автоналив")).color(gh::Colors::Green).value(mode_fill);
    b.LED_("ld_phylamp").label(F("фитолампа")).color(gh::Colors::Green).value(mode_phylamp);
    b.LED_("ld_framuga").label(F("фрамуга")).color(gh::Colors::Green).value(mode_framuga);
  }

  {
    gh::Row r(b);
    b.LED_("ld_twice_week").label(F("2 раза в неделю")).color(gh::Colors::Green).value(mode_twice_week);
    b.LED_("ld_weekly").label(F("еженедельно")).color(gh::Colors::Green).value(mode_weekly);
  }

  // Переключатели режимов через GyverHub
  {
    gh::Row r(b);
    b.Switch_("sw_greenhouse", &mode_greenhouse).label(F("Теплица"));
    b.Switch_("sw_drip", &mode_drip).label(F("Капельный"));
    b.Switch_("sw_sprinkler", &mode_sprinkler).label(F("Спринклер"));
  }
  {
    gh::Row r(b);
    b.Switch_("sw_fill", &mode_fill).label(F("Автоналив"));
    b.Switch_("sw_phylamp", &mode_phylamp).label(F("Фитолампа"));
    b.Switch_("sw_framuga", &mode_framuga).label(F("Фрамуга"));
  }
  {
    gh::Row r(b);
    b.Switch_("sw_twice_week", &mode_twice_week).label(F("2 раза/нед"));
    b.Switch_("sw_weekly", &mode_weekly).label(F("1 раз/нед"));
  }

if (b.beginRow()) {
    b.Label(" краны и клапана");
    b.endRow();
  }
// Виджеты состояния запорной арматуры (входы TCA9555)
  {
    gh::Row r(b);
    b.Label_(F("lbl_input_main")).label(F("гл.кран")).value(inputMainValve ? "открыт" : "закрыт").color(inputMainValve ? gh::Colors::Green : gh::Colors::Red);
    b.Label_(F("lbl_input_gh")).label(F("кл.теплицы")).value(inputGreenhouse ? "открыт" : "закрыт").color(inputGreenhouse ? gh::Colors::Green : gh::Colors::Red);
    b.Label_(F("lbl_input_drip")).label(F("кл.капельный")).value(inputDrip ? "открыт" : "закрыт").color(inputDrip ? gh::Colors::Green : gh::Colors::Red);
  }

  {
    gh::Row r(b);
    b.Label_(F("lbl_input_dir1")).label(F("кл.напр.1")).value(inputDir1 ? "открыт" : "закрыт").color(inputDir1 ? gh::Colors::Green : gh::Colors::Red);
    b.Label_(F("lbl_input_dir2")).label(F("кл.напр.2")).value(inputDir2 ? "открыт" : "закрыт").color(inputDir2 ? gh::Colors::Green : gh::Colors::Red);
  }
  // Температура и влажность
  if (b.beginRow()) {
    b.Label("температура").fontSize(20);
    b.Title_(F("tempr"));
    b.endRow();
  }
  if (b.beginRow()) {
    b.Label("влажность").fontSize(20);
    b.Title_(F("humid"));
    b.endRow();
  }
  if (b.beginRow()) {
    b.Label("t почвы").fontSize(20);
    b.Title_(F("soil_temp"));
    b.endRow();
  }

  // Уровень бака
  if (b.beginRow()) {
    b.Label("вода в баке");
    b.endRow();
  }
  if (b.beginRow()) {
    b.Label("уровень см").fontSize(20);
    b.Title_(F("tank"));
    b.endRow();
  }

  // Canvas уровень бака
  if (b.beginRow()) {
    gh::Canvas cv0;
    b.BeginCanvas_(F("cv0"), 100, 100, &cv0);
    cv0.noStroke();
    cv0.fill(gh::Color(0, 0, 255));   // синий — воздух (фон)
    cv0.rect(0, 0, 100, 100);
    // красная рамка
    cv0.stroke(0xff0000);
    cv0.strokeWeight(5);
    cv0.noFill();
    cv0.rect(0, 0, 100, 100);
    b.EndCanvas();
    b.endRow();
  }
// === НОВЫЕ ВИДЖЕТЫ: Кнопки управления историей и график ===
  if (b.beginRow()) {
    b.Button(&btn_show_graph).label(F("Показать график")).text("Загрузить");
    if (b.click()) {
      graphRequested = true;
      Serial.println("Запрошен график");
    }
    b.Button(&btn_clear_graph).label(F("Очистить историю")).text("Очистить");
    if (b.click()) {
      clearHistoryData();
      Serial.println("История очищена по кнопке");
    }
    b.endRow();
  }

  // График температуры и влажности (canvas-диаграмма)
  if (b.beginRow()) {
    gh::Canvas cv_graph;
    b.BeginCanvas_(F("cv_graph"), 320, 160, &cv_graph);
    // Рисуем диаграмму: t — красный, h — зелёный
    drawChart(cv_graph, graphData, 320, 160);
    b.EndCanvas();
    b.endRow();
  }

// === ВИДЖЕТЫ ПРОГНОЗА ПОГОДЫ (ОТКЛЮЧЕНО) ===
   if (b.beginRow()) {
     b.Button(&btn_weather_forecast).label(F("Прогноз погоды")).text("Загрузить");
     if (b.click()) {
       weatherRequested = true;
       Serial.println("Запрошен прогноз погоды");
     }
     b.endRow();
   }

   if (b.beginRow()) {
     b.Text("weather_display").label(F("Прогноз на 7 дней")).size(5,1).rows(20).align(gh::Align::Left).fontSize(8).value(weatherForecast);
     b.endRow();
   }

}
// =================== РЕЖИМЫ РАБОТЫ ===================

// Проверка дня недели для режимов
bool isDayOfWeek(byte dayOfWeek) {
  // ntp.weekDay() возвращает 1-7 (понедельник-воскресенье)
  // Для простоты: 2 раза в неделю - среда(3) и суббота(6)
  // Еженедельно - воскресенье(7)
  return ntp.weekDay() == dayOfWeek;
}

// Режим 0: Полив теплицы
void runGreenhouse() {
  if (!mode_greenhouse) return;

  // Запрет при закрытом главном кране
  if (!inputMainValve) return;

  // Запрет при сухой почве в теплице
  if (k_greenhouse == 1) return;

  // Запрет при низком уровне бака
  if (cm > 75) return;

  static byte step = 0;

  if (ntp.hour() == y_greenhouse && ntp.minute() == 0 && step == 0) {
    step = 1;
    tmr_process.setTime((unsigned long)duration_greenhouse * 60000);
    tmr_process.startTimeout();
    // Включаем клапан теплицы (самотёк, насос не нужен)
    TCA9555_setRelay(TCA_RELAY_GREENHOUSE, true);
    flag_greenhouse = true;
    lcdState = LCD_ACTIVE_MODE;
    lcdSetCursor(0, 0);
    lcdPrint("poliv teplici ");
  }

  if (step == 1) {
    // Работаем 30 минут
    if (tmr_process) {
      TCA9555_setRelay(TCA_RELAY_GREENHOUSE, false);
      flag_greenhouse = false;
      step = 0;
      lcdState = LCD_TIME;
    }
  }
}

// Режим 1: Капельный полив
void runDrip() {
  if (!mode_drip) return;

  // Запрет при закрытом главном кране
  if (!inputMainValve) return;

  // Запрет при влажности почвы k==1 (сухо)
  if (k == 1) return;

  // Запрет при низком уровне бака (ниже 15 см — осадок на дне)
  if (cm > 85) return;

  // Проверка расписания
  // mode_twice_week — только среда(3) и суббота(6)
  // mode_weekly — только воскресенье(7)
  // Если оба выключены — ежедневно (как сейчас)
  if (mode_twice_week && !isDayOfWeek(3) && !isDayOfWeek(6)) return;
  if (mode_weekly && !isDayOfWeek(7)) return;

  static unsigned long processStart = 0;
  static byte step = 0;

  // Авария потока — экстренная остановка
  if (flowAlarm) { step = 0; return; }

  if (ntp.hour() == y_drip && ntp.minute() == 0 && step == 0) {
    step = 1;
    tmr_process.setTime(10000);
    tmr_process.startTimeout();
    // Включаем клапан капельного и фазу на 10 сек
    TCA9555_setRelay(TCA_RELAY_DRIP, true);
    TCA9555_setRelay(TCA_RELAY_PHASE, true);
    flag_drip = true;
    flag_phase = true;
    lcdState = LCD_ACTIVE_MODE;
    lcdSetCursor(0, 0);
    lcdPrint("kapel poliv");
  }

  if (step == 1) {
    // Через 10 сек выключаем фазу и клапан, включаем насос
    if (tmr_process) {
      TCA9555_setRelay(TCA_RELAY_DRIP, false);
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      TCA9555_setRelay(TCA_RELAY_PUMP, true);
      flag_drip = false;
      flag_phase = false;
      flag_pump = true;
      tmr_pump.setTime(10 * 60000);
      tmr_pump.startTimeout();
      tmr_flow.setTime(3000);
      tmr_flow.startTimeout();
      flowDetected = false;
      tmr_process.setTime(10 * 60000);
      tmr_process.startTimeout();
      step = 2;
    }
  }

  if (step == 2) {
    // Проверка потока через 3 сек после включения насоса
    if (tmr_flow && !flowDetected) {
      SensorPoliv.read();
      d_poliv = SensorPoliv.getFlowRate_m();
      if (d_poliv < 0.5) {  // Нет потока
        TCA9555_setRelay(TCA_RELAY_PUMP, false);
        flag_pump = false;
        step = 0;
        lcdState = LCD_TIME;
        hub.update("ld_alarm_flow").value(1);
        return;
      }
      flowDetected = true;
    }

    // Насос работает 10 минут
    if (tmr_pump) {
      TCA9555_setRelay(TCA_RELAY_PUMP, false);
      flag_pump = false;
      // Открываем клапан капельного на 10 сек (сброс давления)
      TCA9555_setRelay(TCA_RELAY_DRIP, true);
      flag_drip = true;
      tmr_phase.setTime(10000);
      tmr_phase.startTimeout();
      step = 3;
    }
  }

  if (step == 3) {
    // Через 10 сек закрываем клапан
    if (tmr_phase) {
      TCA9555_setRelay(TCA_RELAY_DRIP, false);
      flag_drip = false;
      step = 0;
      lcdState = LCD_TIME;
    }
  }
}

// Режим 2: Спринклерный полив (3 направления)
void runSprinkler() {
  if (!mode_sprinkler) return;

  // Запрет при закрытом главном кране
  if (!inputMainValve) return;

  // Запрет при влажности почвы k==1 (сухо)
  if (k == 1) return;

  // Запрет при низком уровне бака (ниже 15 см — осадок на дне)
  if (cm > 85) return;

  // Проверка расписания
  // mode_twice_week — только среда(3) и суббота(6)
  // mode_weekly — только воскресенье(7)
  // Если оба выключены — ежедневно (как сейчас)
  if (mode_twice_week && !isDayOfWeek(3) && !isDayOfWeek(6)) return;
  if (mode_weekly && !isDayOfWeek(7)) return;

  static byte step = 0;
  static byte direction = 0;

  // Авария потока — экстренная остановка
  if (flowAlarm) { step = 0; direction = 0; return; }

  if (ntp.hour() == y_sprinkler && ntp.minute() == 0 && step == 0) {
    step = 1;
    direction = 1;
  }

  // Направление 1
  if (step == 1 && direction == 1) {
    if (!flag_dir1) {
      TCA9555_setRelay(TCA_RELAY_DIR1, true);
      TCA9555_setRelay(TCA_RELAY_PHASE, true);
      flag_dir1 = true;
      flag_phase = true;
      tmr_dir.setTime(10000);
      tmr_dir.startTimeout();
      lcdState = LCD_ACTIVE_MODE;
      lcdSetCursor(0, 0);
      lcdPrint("sprinkler 1 naprav");
    }
    if (tmr_dir) {
      TCA9555_setRelay(TCA_RELAY_DIR1, false);
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      TCA9555_setRelay(TCA_RELAY_PUMP, true);
      flag_dir1 = false;
      flag_phase = false;
      flag_pump = true;
      tmr_pump.setTime(5 * 60000);
      tmr_pump.startTimeout();
      tmr_flow.setTime(3000);
      tmr_flow.startTimeout();
      flowDetected = false;
      step = 2;
    }
  }

  if (step == 2 && direction == 1) {
    // Проверка потока
    if (tmr_flow && !flowDetected) {
      SensorPoliv.read();
      d_poliv = SensorPoliv.getFlowRate_m();
      if (d_poliv < 0.5) {
        TCA9555_setRelay(TCA_RELAY_PUMP, false);
        flag_pump = false;
        step = 0;
        lcdState = LCD_TIME;
        hub.update("ld_alarm_flow").value(1);
        return;
      }
      flowDetected = true;
    }

    if (tmr_pump) {
      TCA9555_setRelay(TCA_RELAY_PUMP, false);
      flag_pump = false;
      // Открываем DIR1 на 10 сек (сброс давления в линии 1)
      TCA9555_setRelay(TCA_RELAY_DIR1, true);
      flag_dir1 = true;
      tmr_phase.setTime(10000);
      tmr_phase.startTimeout();
      step = 3;
    }
  }

  if (step == 3 && direction == 1) {
    if (tmr_phase) {
      TCA9555_setRelay(TCA_RELAY_DIR1, false);
      flag_dir1 = false;
      direction = 2;
      step = 4;
    }
  }

  // Направление 2
  if (step == 4 && direction == 2) {
    if (!flag_dir2) {
      TCA9555_setRelay(TCA_RELAY_DIR2, true);
      TCA9555_setRelay(TCA_RELAY_PHASE, true);
      flag_dir2 = true;
      flag_phase = true;
      tmr_dir.setTime(10000);
      tmr_dir.startTimeout();
      lcdSetCursor(0, 0);
      lcdPrint("sprinkler 2 naprav");
    }
    if (tmr_dir) {
      TCA9555_setRelay(TCA_RELAY_DIR2, false);
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      TCA9555_setRelay(TCA_RELAY_PUMP, true);
      flag_dir2 = false;
      flag_phase = false;
      flag_pump = true;
      tmr_pump.setTime(5 * 60000);
      tmr_pump.startTimeout();
      tmr_flow.setTime(3000);
      tmr_flow.startTimeout();
      flowDetected = false;
      step = 5;
    }
  }

  if (step == 5 && direction == 2) {
    if (tmr_flow && !flowDetected) {
      SensorPoliv.read();
      d_poliv = SensorPoliv.getFlowRate_m();
      if (d_poliv < 0.5) {
        TCA9555_setRelay(TCA_RELAY_PUMP, false);
        flag_pump = false;
        step = 0;
        lcdState = LCD_TIME;
        hub.update("ld_alarm_flow").value(1);
        return;
      }
      flowDetected = true;
    }

    if (tmr_pump) {
      TCA9555_setRelay(TCA_RELAY_PUMP, false);
      flag_pump = false;
      // Открываем DIR2 на 10 сек (сброс давления в линии 2)
      TCA9555_setRelay(TCA_RELAY_DIR2, true);
      flag_dir2 = true;
      tmr_phase.setTime(10000);
      tmr_phase.startTimeout();
      step = 6;
    }
  }

  if (step == 6 && direction == 2) {
    if (tmr_phase) {
      TCA9555_setRelay(TCA_RELAY_DIR2, false);
      flag_dir2 = false;
      direction = 3;
      step = 7;
    }
  }

  // Направление 3
  if (step == 7 && direction == 3) {
    if (!flag_dir3) {
      TCA9555_setRelay(TCA_RELAY_DIR3, true);
      TCA9555_setRelay(TCA_RELAY_PHASE, true);
      flag_dir3 = true;
      flag_phase = true;
      tmr_dir.setTime(10000);
      tmr_dir.startTimeout();
      lcdSetCursor(0, 0);
      lcdPrint("sprinkler 3 naprav");
    }
    if (tmr_dir) {
      TCA9555_setRelay(TCA_RELAY_DIR3, false);
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      TCA9555_setRelay(TCA_RELAY_PUMP, true);
      flag_dir3 = false;
      flag_phase = false;
      flag_pump = true;
      tmr_pump.setTime(5 * 60000);
      tmr_pump.startTimeout();
      tmr_flow.setTime(3000);
      tmr_flow.startTimeout();
      flowDetected = false;
      step = 8;
    }
  }

  if (step == 8 && direction == 3) {
    if (tmr_flow && !flowDetected) {
      SensorPoliv.read();
      d_poliv = SensorPoliv.getFlowRate_m();
      if (d_poliv < 0.5) {
        TCA9555_setRelay(TCA_RELAY_PUMP, false);
        flag_pump = false;
        step = 0;
        lcdState = LCD_TIME;
        hub.update("ld_alarm_flow").value(1);
        return;
      }
      flowDetected = true;
    }

    if (tmr_pump) {
      TCA9555_setRelay(TCA_RELAY_PUMP, false);
      flag_pump = false;
      // Открываем DIR3 на 10 сек (сброс давления в линии 3)
      TCA9555_setRelay(TCA_RELAY_DIR3, true);
      flag_dir3 = true;
      tmr_phase.setTime(10000);
      tmr_phase.startTimeout();
      step = 9;
    }
  }

  if (step == 9 && direction == 3) {
    if (tmr_phase) {
      TCA9555_setRelay(TCA_RELAY_DIR3, false);
      flag_dir3 = false;
      step = 0;
      lcdState = LCD_TIME;
    }
  }
}

// Режим 3: Автоналив
void runAutoFill() {
  if (!mode_fill) return;

  static bool filling = false;

  // Уровень ниже 25 см (100-cm < 25, т.е. cm > 75)
  if (!filling && cm > 75) {
    filling = true;
    // Включаем клапан налива
    digitalWrite(relayFillPin, HIGH);
    flag_fill = true;
    lcdState = LCD_ACTIVE_MODE;
    lcdSetCursor(0, 0);
    lcdPrint("avtonaliv ");
  }

  // Уровень достиг 80 см (100-cm >= 80, т.е. cm <= 20)
  if (filling && cm <= 20) {
    filling = false;
    digitalWrite(relayFillPin, LOW);
    flag_fill = false;
    lcdState = LCD_TIME;
  }
}

// Режим 4: Фитолампа
void runPhylamp() {
  if (!mode_phylamp) return;

  static unsigned long processStart = 0;
  static bool running = false;

  if (ntp.hour() == y_phylamp && ntp.minute() == 0 && !running) {
    running = true;
    tmr_process.setTime(4 * 3600000UL);
    tmr_process.startTimeout();
    // Включаем фитолампу (LOW = ON)
    digitalWrite(relayPhylampPin, LOW);
    flag_phylamp = true;
    lcdState = LCD_ACTIVE_MODE;
    lcdSetCursor(0, 0);
    lcdPrint("fitolampa  ");
  }

  if (running) {
    // Работаем 4 часа
    if (tmr_process) {
      digitalWrite(relayPhylampPin, HIGH); // OFF
      flag_phylamp = false;
      running = false;
      lcdState = LCD_TIME;
    }
  }
}

// Режим 5: Фрамуга
void runFramuga() {
  if (!mode_framuga) return;

  // t > 35 - включаем
  if (t > 35 && !flag_framuga) {
    TCA9555_setRelay(TCA_RELAY_FRAMUGA, true);
    flag_framuga = true;
    lcdState = LCD_ACTIVE_MODE;
    lcdSetCursor(0, 0);
    lcdPrint("framuga otkr ");
  }

  // t <= 20 - выключаем
  if (t <= 20 && flag_framuga) {
    TCA9555_setRelay(TCA_RELAY_FRAMUGA, false);
    flag_framuga = false;
    lcdState = LCD_TIME;
  }
}


// =================== LCD ФУНКЦИИ ===================
void lcdDrawLevelBar() {
  char buf[17];

  if (cm < 0) {
    // Датчик не работает
    lcdSetCursor(0, 0);
    lcdPrint("[  OWIBKA     ]");
    lcdSetCursor(0, 1);
    lcdPrint("UROVEN ---%  ");
    return;
  }

  int pct = constrain(100 - cm, 0, 100);
  int barWidth = map(pct, 0, 100, 0, 14);

  // Верхний ряд: [####       ]
  buf[0] = '[';
  for (int i = 0; i < 14; i++) buf[1 + i] = (i < barWidth) ? '#' : ' ';
  buf[15] = ']';
  buf[16] = '\0';
  lcdSetCursor(0, 0);
  lcdPrint(buf);

  // Нижний ряд: УРОВЕНЬ 75%
  snprintf(buf, sizeof(buf), "UROVEN %3d%%  ", pct);
  lcdSetCursor(0, 1);
  lcdPrint(buf);
}

void lcdShowTime() {
  lcdSetCursor(0, 0);
  lcdPrint(stamp_2);
  lcdSetCursor(0, 1);
  if (cm > 80) {  // уровень ниже 20%
    lcdPrint("ALARM UROVEN! ");
  } else {
    lcdPrint(stamp_1);
  }
}

void lcdShowModeSelect() {
  lcdSetCursor(0, 0);
  lcdPrint("vibor regima:  ");
  lcdSetCursor(0, 1);
  switch (j) {
    case 0: lcdPrint("teplica     "); break;
    case 1: lcdPrint("kapelni     "); break;
    case 2: lcdPrint("sprinkl     "); break;
    case 3: lcdPrint("avtnaliv    "); break;
    case 4: lcdPrint("fitolam     "); break;
    case 5: lcdPrint("framuga    "); break;
  }
  // Показываем состояние выбранного режима
  bool modeOn = false;
  switch (j) {
    case 0: modeOn = mode_greenhouse; break;
    case 1: modeOn = mode_drip; break;
    case 2: modeOn = mode_sprinkler; break;
    case 3: modeOn = mode_fill; break;
    case 4: modeOn = mode_phylamp; break;
    case 5: modeOn = mode_framuga; break;
  }
  lcdPrint(modeOn ? " on " : " off");
}

// =================== ФУНКЦИЯ ОТРИСОВКИ ГРАФИКА (CANVAS) ===================
void drawChart(gh::Canvas& cv, const String& data, int w, int h) {
  const int MARGIN_L = 35, MARGIN_R = 8, MARGIN_T = 8, MARGIN_B = 20;
  int drawW = w - MARGIN_L - MARGIN_R;
  int drawH = h - MARGIN_T - MARGIN_B;

  // Парсинг данных
  float days[100], temps[100], hums[100];
  int count = 0;
  int idx = 0;
  while (idx < (int)data.length() && count < 100) {
    int nextPipe = data.indexOf('|', idx);
    if (nextPipe < 0) nextPipe = data.length();
    String line = data.substring(idx, nextPipe);

    int c1 = line.indexOf(',');
    int c2 = line.indexOf(',', c1 + 1);
    if (c1 > 0 && c2 > 0) {
      days[count] = line.substring(0, c1).toFloat();
      temps[count] = line.substring(c1 + 1, c2).toFloat();
      hums[count] = line.substring(c2 + 1).toFloat();
      count++;
    }
    idx = nextPipe + 1;
    if (nextPipe == (int)data.length()) break;
  }

  if (count < 2) {
    cv.background();
    return;
  }

  // Определяем диапазоны
  float minDay = days[0], maxDay = days[0];
  float minTemp = 99, maxTemp = -99;
  float minHum = 99, maxHum = -99;
  for (int i = 0; i < count; i++) {
    if (days[i] < minDay) minDay = days[i];
    if (days[i] > maxDay) maxDay = days[i];
    if (temps[i] < minTemp) minTemp = temps[i];
    if (temps[i] > maxTemp) maxTemp = temps[i];
    if (hums[i] < minHum) minHum = hums[i];
    if (hums[i] > maxHum) maxHum = hums[i];
  }
  if (minTemp > maxTemp) { minTemp = 0; maxTemp = 50; }
  if (minHum > maxHum) { minHum = 0; maxHum = 100; }
  // Добавляем отступы по 5% для наглядности
  float padTemp = (maxTemp - minTemp) * 0.05;
  float padHum  = (maxHum  - minHum)  * 0.05;
  if (padTemp < 1) padTemp = 2;
  if (padHum  < 1) padHum  = 5;
  minTemp -= padTemp; maxTemp += padTemp;
  minHum  -= padHum;  maxHum  += padHum;

  // Преобразование координат
  auto mapX = [&](float d) -> int {
    return MARGIN_L + (int)((d - minDay) / (maxDay - minDay) * drawW);
  };
  auto mapY_temp = [&](float t) -> int {
    return MARGIN_T + drawH - (int)((t - minTemp) / (maxTemp - minTemp) * drawH);
  };
  auto mapY_hum = [&](float h_) -> int {
    return MARGIN_T + drawH - (int)((h_ - minHum) / (maxHum - minHum) * drawH);
  };

  cv.background();
  cv.strokeWeight(1);

  // Сетка Y
  cv.stroke(0x333333);
  for (int y = 0; y <= 4; y++) {
    int yy = MARGIN_T + drawH * y / 4;
    cv.line(MARGIN_L, yy, w - MARGIN_R, yy);
  }

  // Пара линий: верха рамки и оси Y
  cv.stroke(0x888888);
  cv.strokeWeight(2);
  cv.line(MARGIN_L, MARGIN_T, MARGIN_L, h - MARGIN_B);                // Y
  cv.line(MARGIN_L, h - MARGIN_B, w - MARGIN_R, h - MARGIN_B);        // X
  cv.line(MARGIN_L, MARGIN_T, w - MARGIN_R, MARGIN_T);                // верх
  cv.line(w - MARGIN_R, MARGIN_T, w - MARGIN_R, h - MARGIN_B);        // право

  // Температура — красная линия
  cv.stroke(0xFF3333);
  cv.strokeWeight(2);
  for (int i = 1; i < count; i++) {
    int x1 = mapX(days[i - 1]), y1 = mapY_temp(temps[i - 1]);
    int x2 = mapX(days[i]),     y2 = mapY_temp(temps[i]);
    cv.line(x1, y1, x2, y2);
  }
  // Точки температуры
  cv.stroke(0xFF0000);
  cv.strokeWeight(4);
  for (int i = 0; i < count; i++) {
    int x = mapX(days[i]), y = mapY_temp(temps[i]);
    cv.line(x - 1, y - 1, x + 1, y + 1);
    cv.line(x - 1, y + 1, x + 1, y - 1);
  }

  // Влажность — зелёная линия
  cv.stroke(0x33CC33);
  cv.strokeWeight(2);
  for (int i = 1; i < count; i++) {
    int x1 = mapX(days[i - 1]), y1 = mapY_hum(hums[i - 1]);
    int x2 = mapX(days[i]),     y2 = mapY_hum(hums[i]);
    cv.line(x1, y1, x2, y2);
  }
  // Точки влажности
  cv.stroke(0x00AA00);
  cv.strokeWeight(4);
  for (int i = 0; i < count; i++) {
    int x = mapX(days[i]), y = mapY_hum(hums[i]);
    cv.line(x - 2, y, x + 2, y);
    cv.line(x, y - 2, x, y + 2);
  }

  // Легенда
  cv.stroke(0xFF3333); cv.strokeWeight(6);
  cv.line(MARGIN_L + 4, MARGIN_T + 6, MARGIN_L + 24, MARGIN_T + 6);
  cv.stroke(0x33CC33);
  cv.line(MARGIN_L + 4, MARGIN_T + 16, MARGIN_L + 24, MARGIN_T + 16);
}

// =================== NVS СОХРАНЕНИЕ НАСТРОЕК ===================
Preferences prefs;

// Предыдущие значения для отслеживания изменений
bool prev_mode_greenhouse = false;
bool prev_mode_drip = false;
bool prev_mode_sprinkler = false;
bool prev_mode_fill = false;
bool prev_mode_phylamp = false;
bool prev_mode_framuga = false;
bool prev_mode_twice_week = false;
bool prev_mode_weekly = false;
int prev_y_greenhouse = 6;
int prev_y_drip = 7;
int prev_y_sprinkler = 8;
int prev_y_phylamp = 18;
int prev_y_twice_week = 9;
int prev_y_weekly = 10;
int prev_duration_greenhouse = 30;
int prev_duration_drip = 10;
int prev_duration_sprinkler = 15;

void saveSettings() {
  prefs.begin("modes", false);
  prefs.putBool("mode_gh", mode_greenhouse);
  prefs.putBool("mode_drip", mode_drip);
  prefs.putBool("mode_spr", mode_sprinkler);
  prefs.putBool("mode_fill", mode_fill);
  prefs.putBool("mode_phyl", mode_phylamp);
  prefs.putBool("mode_fram", mode_framuga);
  prefs.putBool("mode_2w", mode_twice_week);
  prefs.putBool("mode_1w", mode_weekly);
  prefs.putInt("y_gh", y_greenhouse);
  prefs.putInt("y_drip", y_drip);
  prefs.putInt("y_spr", y_sprinkler);
  prefs.putInt("y_phyl", y_phylamp);
  prefs.putInt("y_2w", y_twice_week);
  prefs.putInt("y_1w", y_weekly);
  prefs.putInt("dur_gh", duration_greenhouse);
  prefs.putInt("dur_drip", duration_drip);
  prefs.putInt("dur_spr", duration_sprinkler);
  prefs.end();
  Serial.println("Settings saved to NVS");
}

void loadSettings() {
  prefs.begin("modes", true);
  mode_greenhouse = prefs.getBool("mode_gh", false);
  mode_drip = prefs.getBool("mode_drip", false);
  mode_sprinkler = prefs.getBool("mode_spr", false);
  mode_fill = prefs.getBool("mode_fill", false);
  mode_phylamp = prefs.getBool("mode_phyl", false);
  mode_framuga = prefs.getBool("mode_fram", false);
  mode_twice_week = prefs.getBool("mode_2w", false);
  mode_weekly = prefs.getBool("mode_1w", false);
  y_greenhouse = prefs.getInt("y_gh", 6);
  y_drip = prefs.getInt("y_drip", 7);
  y_sprinkler = prefs.getInt("y_spr", 8);
  y_phylamp = prefs.getInt("y_phyl", 18);
  y_twice_week = prefs.getInt("y_2w", 9);
  y_weekly = prefs.getInt("y_1w", 10);
  duration_greenhouse = prefs.getInt("dur_gh", 30);
  duration_drip = prefs.getInt("dur_drip", 10);
  duration_sprinkler = prefs.getInt("dur_spr", 15);
  prefs.end();

  // Синхронизируем prev-переменные после загрузки
  prev_mode_greenhouse = mode_greenhouse;
  prev_mode_drip = mode_drip;
  prev_mode_sprinkler = mode_sprinkler;
  prev_mode_fill = mode_fill;
  prev_mode_phylamp = mode_phylamp;
  prev_mode_framuga = mode_framuga;
  prev_mode_twice_week = mode_twice_week;
  prev_mode_weekly = mode_weekly;
  prev_y_greenhouse = y_greenhouse;
  prev_y_drip = y_drip;
  prev_y_sprinkler = y_sprinkler;
  prev_y_phylamp = y_phylamp;
  prev_y_twice_week = y_twice_week;
  prev_y_weekly = y_weekly;
  prev_duration_greenhouse = duration_greenhouse;
  prev_duration_drip = duration_drip;
  prev_duration_sprinkler = duration_sprinkler;

  Serial.println("Settings loaded from NVS");
}

// Проверка изменений настроек и автосохранение в NVS
void checkSettingsChanged() {
  if (mode_greenhouse != prev_mode_greenhouse ||
      mode_drip != prev_mode_drip ||
      mode_sprinkler != prev_mode_sprinkler ||
      mode_fill != prev_mode_fill ||
      mode_phylamp != prev_mode_phylamp ||
      mode_framuga != prev_mode_framuga ||
      mode_twice_week != prev_mode_twice_week ||
      mode_weekly != prev_mode_weekly ||
      y_greenhouse != prev_y_greenhouse ||
      y_drip != prev_y_drip ||
      y_sprinkler != prev_y_sprinkler ||
      y_phylamp != prev_y_phylamp ||
      y_twice_week != prev_y_twice_week ||
      y_weekly != prev_y_weekly ||
      duration_greenhouse != prev_duration_greenhouse ||
      duration_drip != prev_duration_drip ||
      duration_sprinkler != prev_duration_sprinkler) {
    saveSettings();
    // Обновляем prev-переменные после сохранения
    prev_mode_greenhouse = mode_greenhouse;
    prev_mode_drip = mode_drip;
    prev_mode_sprinkler = mode_sprinkler;
    prev_mode_fill = mode_fill;
    prev_mode_phylamp = mode_phylamp;
    prev_mode_framuga = mode_framuga;
    prev_mode_twice_week = mode_twice_week;
    prev_mode_weekly = mode_weekly;
    prev_y_greenhouse = y_greenhouse;
    prev_y_drip = y_drip;
    prev_y_sprinkler = y_sprinkler;
    prev_y_phylamp = y_phylamp;
    prev_y_twice_week = y_twice_week;
    prev_y_weekly = y_weekly;
    prev_duration_greenhouse = duration_greenhouse;
    prev_duration_drip = duration_drip;
    prev_duration_sprinkler = duration_sprinkler;
  }
}

void setup() {
  Serial.begin(115200);

  // ========== ДИАГНОСТИКА ПРИЧИНЫ ПЕРЕЗАГРУЗКИ ==========
  // Выводим причину последнего сброса — это сразу покажет, watchdog,
  // brownout (питание) или crash виноваты в перезагрузке.
  esp_reset_reason_t rst = esp_reset_reason();
  Serial.print("Reset reason: ");
  switch (rst) {
    case ESP_RST_POWERON:     Serial.println("POWERON (питание включено)"); break;
    case ESP_RST_EXT:         Serial.println("EXT (кнопка EN/reset)"); break;
    case ESP_RST_SW:          Serial.println("SW (программный сброс)"); break;
    case ESP_RST_PANIC:       Serial.println("PANIC (crash/исключение)"); break;
    case ESP_RST_INT_WDT:     Serial.println("INT_WDT (прерывание-сторож)"); break;
    case ESP_RST_TASK_WDT:    Serial.println("TASK_WDT (loop завис > 5 сек)"); break;
    case ESP_RST_WDT:         Serial.println("WDT (сторож)"); break;
    case ESP_RST_DEEPSLEEP:   Serial.println("DEEPSLEEP"); break;
    case ESP_RST_BROWNOUT:    Serial.println("BROWNOUT (питание просело!)"); break;
    case ESP_RST_SDIO:        Serial.println("SDIO"); break;
    default:                  Serial.println((int)rst); break;
  }

  // ========== WATCHDOG ==========
  // TWDT уже инициализирован Arduino-ядром (таймаут ~5 сек).
  // Добавляем текущую задачу (loop) под надзор и сбрасываем таймер в каждом loop().
  // Если loop() зависнет более чем на ~5 сек — ESP32 перезагрузится.
  esp_task_wdt_add(NULL);       // Добавляем текущую задачу (loop) под надзор

  // Инициализация пинов
  pinMode(manPin, INPUT_PULLUP);
  pinMode(fillPin, INPUT_PULLUP);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(gruntPin, INPUT);
  pinMode(gruntGreenhousePin, INPUT);
  pinMode(btnModePin, INPUT_PULLUP);
  pinMode(btnConfirmPin, INPUT_PULLUP);
  pinMode(relayPhylampPin, OUTPUT);
  pinMode(relayFillPin, OUTPUT);

  // runPhylamp: LOW = ON, HIGH = OFF (инвертировано)
  digitalWrite(relayPhylampPin, HIGH);
  digitalWrite(relayFillPin, LOW);

  attachInterrupt(digitalPinToInterrupt(btnModePin), btnModeISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnConfirmPin), btnConfirmISR, FALLING);

Wire.begin();
  Wire.setTimeOut(100); // таймаут I2C 100ms, чтобы не вешать шину
  TCA9555_init();
  TCA9555_allOff();

  // Датчики
  dht.begin();
  soilSensor.setResolution(12);  // Датчик температуры почвы
  SensorPoliv.begin(countPoliv);
  SensorNaliv.begin(countNaliv);

  // LCD
  lcdInit();
  lcdBacklight();
  lcdClear();
  lcdPrint("Огород v2.1");
  delay(2000);

  #ifdef GH_ESP_BUILD
  WiFi.mode(WIFI_STA);
  IPAddress local_IP(192, 168, 0, 110);
  IPAddress gateway(192, 168, 0, 1);
  IPAddress subnet(255, 255, 255, 0);
  IPAddress dns(8, 8, 8, 8);
  WiFi.config(local_IP, gateway, subnet, dns);
  WiFi.begin(AP_SSID, AP_PASS);
  lcdClear();
  lcdPrint("WiFi...");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 30000) {
    delay(500);
    esp_task_wdt_reset();  // страховка watchdog при долгом коннекте
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\nWiFi не подключился за 30 сек, продолжаем без сети");
    lcdClear();
    lcdPrint("WiFi net");
    delay(2000);
  } else {
    Serial.println();
    Serial.println(WiFi.localIP());

    // Показываем IP на LCD ~60 сек, потом переключится на время
    lcdClear();
    lcdPrint("IP:");
    lcdSetCursor(0, 1);
    ipAddr = WiFi.localIP().toString();
    lcdPrint(ipAddr);
    showIpUntil = millis() + 60000;
  }

  hub.mqtt.config("m9.wqtt.ru", 20042, "u_F84LY5", "rdu4PozQ");
  hub.setVersion("AndyInjiner/Ogorod@2.1");
  #else
    // Без WiFi — просто пауза
    delay(3000);
  #endif

  // GyverHub
  data["key0"] = "дождь";
  data["key1"] = "сухо";
  data["key2"] = "почва высохла";
  data["key3"] = "почва влажная";
  data["key4"] = "почва мокрая";

  hub.config(F("MyDevices"), F("OGOROD"), F(""));
  hub.onBuild(build);

  // Бот
  //bot.attachUpdate(updateh);
  //bot.setToken(F(BOT_TOKEN));
  //bot.setPollMode(fb::Poll::Long, 20000);
  //bot.setLimit(1);

  //fb::Message msg("Меню управления", CHAT_ID);
  //fb::InlineMenu menu("restart;status;fill\nstop_fill;test", "n_1;n_2;n_3;n_4;n_5");
  //msg.setInlineMenu(menu);
  //bot.sendMessage(msg);
  //bot.sendMessage(fb::Message("система запущена", CHAT_ID));

  ntp.begin();
  hub.begin();
  data.begin();
  loadSettings();  // Восстанавливаем режимы и настройки времени из NVS
}
// =================== ДАТЧИКИ ===================
void readSensors() {
  // DHT11
  h = dht.readHumidity();
  t = dht.readTemperature();

  // Ультразвуковой датчик уровня
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  // Измеряем вручную через micros() — надёжнее pulseIn на ESP32
  { unsigned long t0 = micros();
    unsigned long tout = t0 + 30000;
    while (digitalRead(PIN_ECHO) == LOW && micros() < tout);
    if (micros() >= tout) {
      cmFailCnt++;                // ошибка — увеличиваем счётчик
      if (cmFailCnt >= 3) {       // 3 ошибки подряд — сбрасываем
        cmFilt = -1;
        cm = -1;
        Serial.println("ECHO lost");
      }
    } else {
      t0 = micros();
      tout = t0 + 25000;
      while (digitalRead(PIN_ECHO) == HIGH && micros() < tout);
      cmFailCnt = 0;              // успех — сбрасываем ошибки
      duration = micros() - t0;
      long rawCm = (duration / 2) / 29.1;
      // Плавное обновление: среднее с предыдущим
      if (cmFilt < 0) cmFilt = rawCm;
      else cmFilt = (cmFilt + rawCm) / 2;
      cm = cmFilt;
      Serial.print("raw="); Serial.print(rawCm);
      Serial.print(" filt="); Serial.println(cmFilt);
    }
  }

  // Датчик влажности почвы (аналоговый, огород)
  g = analogRead(gruntPin);
  k = (g > 750) ? 1 : 0;  // порог: >750 = сухо

  // Датчик влажности почвы (аналоговый, теплица)
  g_greenhouse = analogRead(gruntGreenhousePin);
  k_greenhouse = (g_greenhouse > 750) ? 1 : 0;
  if (k == 1) {
    val = s_2;  // сухая
  } else {
    val = s_1;  // дождь (влажная)
  }

  if (g < 300) {
    val_1 = s_5;  // мокрая
  } else if (g > 350 && g < 750) {
    val_1 = s_4;  // влажная
  } else if (g > 800) {
    val_1 = s_3;  // высохла
  }

  // Датчик температуры почвы DS18B20
  soilSensor.tick();
  soilTemp = soilSensor.getTemp();
}

// =================== LOOP ===================
void loop() {
  esp_task_wdt_reset();  // сброс Watchdog

  // ========== WiFi CHECK ==========
  // Если WiFi нет — пропускаем hub.tick(), чтобы MQTT коннект не заблокировал loop
  if (WiFi.status() == WL_CONNECTED) {
    hub.tick();
    esp_task_wdt_reset();  // MQTT tick может занять время — страховка watchdog
  }
  ntp.tick();
  data.tick();

  // ========== I2C RECOVERY ==========
  // Восстановление при ошибке — не чаще 1 раза в 5 сек
  if (i2cError && millis() - lastI2cRecovery > 5000) {
    Serial.println("I2C error detected, recovery...");
    i2cBusRecovery();
  }

  // Периодическое восстановление I2C шины (раз в 10 мин для профилактики) — ОТКЛЮЧЕНО
  // Профилактика вызывает Wire.end()+Wire.begin() на работающей шине,
  // что на фоне прерываний FlowSensor (до 450 Гц) само провоцирует сбой.
  // Recovery работает только при реальной ошибке i2cError.
  // if (millis() - lastI2cRecovery > I2C_RECOVERY_INTERVAL) {
  //   i2cBusRecovery();
  // }

  // ========== WiFi RECONNECT ==========
  // Если WiFi отвалился — переподключаемся раз в 30 сек
  // GyverHub сам восстановит MQTT после восстановления WiFi
  if (WiFi.status() != WL_CONNECTED) {
    static unsigned long lastWifiReconnect = 0;
    if (millis() - lastWifiReconnect > 30000) {
      Serial.println("WiFi lost, reconnecting...");
      WiFi.reconnect();
      lastWifiReconnect = millis();
    }
  }

  // ========== ДАТЧИКИ (раз в 1 сек) ==========
  static gh::Timer tmr_sensors(1000);
  if (tmr_sensors) {
    readSensors();
    TCA9555_readInputs();
    writeHistoryData();
  }

  // ========== КНОПКИ ==========
  checkButtons();

  // ========== LCD ==========
  if (lcdState != prevLcdState) {
    lcdClear();
    prevLcdState = lcdState;
  }

  static bool lcdTimeChanged = true;
  switch (lcdState) {
    case LCD_TIME:
      if (showIpUntil > 0 && millis() < showIpUntil) {
        // IP уже выведен в setup
      } else {
        showIpUntil = 0;
        if (lcdTimeChanged || lcdState != prevLcdState) {
          lcdShowTime();
          lcdTimeChanged = false;
        }
      }
      break;
    case LCD_LEVEL:
      { static unsigned long lastLvlUpd = 0;
        if (millis() - lastLvlUpd > 2000) {
          lcdDrawLevelBar();
          lastLvlUpd = millis();
        }
      }
      if (tmr_lcd_timeout) {
        lcdState = LCD_TIME;
        lcdClear();
        prevJ = -1;
      }
      break;
    case LCD_MODE_SELECT:
      if (j != prevJ) {
        lcdShowModeSelect();
        prevJ = j;
      }
      if (tmr_lcd_timeout) {
        lcdState = LCD_TIME;
      }
      break;
    case LCD_ACTIVE_MODE:
      break;
  }

  // ========== РЕЖИМЫ РАБОТЫ ==========
  runGreenhouse();
  runDrip();
  runSprinkler();
  runAutoFill();
  runPhylamp();
  runFramuga();
  esp_task_wdt_reset();  // после режимов — страховка watchdog

  // ========== КОНТРОЛЬ ПОТОКА (раз в 30 сек, пока работает насос) ==========
  static gh::Timer tmr_flow_monitor(30000);
  // Автосброс аварии через 10 минут
  if (flowAlarm) {
    static gh::Timer tmr_alarm_clear(600000);
    if (tmr_alarm_clear) {
      flowAlarm = false;
      hub.update("ld_alarm_flow").value(0);
      Serial.println("FLOW ALARM auto-cleared after 10 min");
    }
  }
  if (tmr_flow_monitor && flag_pump) {
    SensorPoliv.read();
    d_poliv = SensorPoliv.getFlowRate_m();
    if (d_poliv < 0.5) {
      // Авария — насос работает, а воды нет
      Serial.println("FLOW ALARM! Pump on but no flow");
      flowAlarm = true;
      flag_pump = false;
      relayState |= 0x00FF;   // все реле выключены (HIGH = OFF)
      flag_drip = flag_dir1 = flag_dir2 = flag_dir3 = flag_phase = false;
      TCA9555_write(relayState);
      digitalWrite(relayFillPin, LOW);
      hub.update("ld_alarm_flow").value(1);
      lcdClear();
      lcdSetCursor(0, 0);
      lcdPrint("AVARIA POTOKA! ");
      lcdSetCursor(0, 1);
      lcdPrint("NASOS VYKL");
    }
  }

  // ========== ТАЙМЕРЫ ОБНОВЛЕНИЙ ==========
  static gh::Timer tmr(3000);
  static gh::Timer tmr_1(10000);
  static gh::Timer tmr_2(20000);
  static gh::Timer tmr_3(50000);

  if (tmr) {
    checkSettingsChanged();  // Автосохранение настроек при изменениях через GyverHub
    hub.update(F("tempr")).value(t);
    hub.update(F("humid")).value(h);
    hub.update(F("soil_temp")).value(soilTemp, 1);
    if (cm >= 0) {
      hub.update(F("tank")).value(100 - cm);
    } else {
      hub.update(F("tank")).value(-1);
    }
    hub.update(F("disp_3")).value(soilTemp, 1);
    hub.update(F("disp_4")).value(soilTemp, 1);
    SensorPoliv.read();
    d_poliv = SensorPoliv.getFlowRate_m();
    SensorNaliv.read();
    d_naliv = SensorNaliv.getFlowRate_m();
    hub.update(F("pot_poliv")).value(d_poliv, 2);
    hub.update(F("pot_naliv")).value(d_naliv, 2);
    hub.update(F("disp_1")).value(val);
    hub.update(F("disp_2")).value(val_1);

    // Canvas уровень бака
    { gh::CanvasUpdate cv0("cv0", &hub);
      cv0.clearRect(0, 0, 100, 100);
      int hAir = constrain(cm, 0, 100);
      int hWat = constrain(100 - cm, 0, 100);
      cv0.noStroke();
      cv0.fill(gh::Color(0, 0, 255));
      cv0.rect(0, 0, 100, hAir);
      cv0.fill(gh::Color(0, 255, 0));
      cv0.rect(0, hAir, 100, hWat);
      cv0.stroke(0xff0000);
      cv0.strokeWeight(5);
      cv0.noFill();
      cv0.rect(0, 0, 100, 100);
      cv0.send();
    }

    hub.update("ld_greenhouse").color(flag_greenhouse ? gh::Colors::Red : gh::Colors::Green).value(mode_greenhouse);
    hub.update("ld_drip").color(flag_pump ? gh::Colors::Red : gh::Colors::Green).value(mode_drip);
    hub.update("ld_sprinkler").color(flag_pump ? gh::Colors::Red : gh::Colors::Green).value(mode_sprinkler);
    hub.update("ld_fill").color(flag_fill ? gh::Colors::Red : gh::Colors::Green).value(mode_fill);
    hub.update("ld_phylamp").color(flag_phylamp ? gh::Colors::Red : gh::Colors::Green).value(mode_phylamp);
    hub.update("ld_framuga").color(flag_framuga ? gh::Colors::Red : gh::Colors::Green).value(mode_framuga);
    hub.update("ld_twice_week").color(gh::Colors::Green).value(mode_twice_week);
    hub.update("ld_weekly").color(gh::Colors::Green).value(mode_weekly);

    hub.update("lbl_input_main").value(inputMainValve ? "открыт" : "закрыт");
    hub.update("lbl_input_gh").value(inputGreenhouse ? "открыт" : "закрыт");
    hub.update("lbl_input_drip").value(inputDrip ? "открыт" : "закрыт");
    hub.update("lbl_input_dir1").value(inputDir1 ? "открыт" : "закрыт");
    hub.update("lbl_input_dir2").value(inputDir2 ? "открыт" : "закрыт");
    if (cm > 75) {
      hub.update("ld_alarm_level").value(1);
    } else {
      hub.update("ld_alarm_level").value(0);
    }
  }

  if (tmr_1) {
    stamp_1 = ntp.timeToString();
    stamp_2 = ntp.dateToString();
    lcdTimeChanged = true;
    hub.update("date").value(stamp_1);
    hub.update("time").value(stamp_2);
  }

  if (tmr_2) {
    if (cm > 75) {
      //bot.sendMessage(fb::Message("бак пустой! уровень критический", CHAT_ID));
    }
  }

  if (tmr_3) {
    hub.sendGet("disp_3", g);
    hub.sendGet("tempr", t);
    hub.sendGet("humid", h);
    hub.sendGet("soil_temp", soilTemp);
    hub.sendGet("pot_poliv", d_poliv);
    hub.sendGet("pot_naliv", d_naliv);
    hub.sendGet("tank", (cm >= 0) ? (100 - cm) : -1);
    hub.sendGet("disp_1", val);
    hub.sendGet("disp_2", val_1);

    if (graphRequested) {
      graphData = readHistoryData();
      graphRequested = false;
      Serial.println("График запрошен, canvas обновится при sendRefresh");
    }
     if (weatherRequested) {
       requestWeatherForecast();
       hub.update("weather_display").value(weatherForecast);
       weatherRequested = false;
       Serial.println("Прогноз погоды отправлен в GyverHub");
     }
    hub.sendRefresh();
  }
}
