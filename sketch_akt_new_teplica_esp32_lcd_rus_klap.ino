/*
 * Скетч управления системой полива "Огород"
 * Версия: 2.0
 * Оборудование: ESP32, LCD1602, TCA9555, датчики
 */

#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <GyverNTP.h>
#include <RCSwitch.h>
#include <Arduino.h>
#include <FlowSensor.h>
#include <GyverHub.h>
#include <FastBot2.h>
#include <PairsFile.h>

// =================== КОНФИГУРАЦИЯ ===================
#define AP_SSID "MERCUSYS_3BCA"
#define AP_PASS "router320"
#define BOT_TOKEN "7358886965:AAH2yQT5gs6qFp4XQHRzkrsbsoLGZlS-ItU"
#define CHAT_ID "5286033464"
#define DHTTYPE DHT11       // Тип датчика DHT

// =================== ПИНЫ ESP32 ===================
#define DHTPIN 17           // Датчик температуры и влажности DHT11
#define PIN_TRIG 5          // Ультразвуковой датчик уровня (TRIG)
#define PIN_ECHO 18         // Ультразвуковой датчик уровня (ECHO)
#define manPin 27           // Датчик потока полив YF-S201
#define fillPin 14          // Датчик потока налив YF-S201 (освобожден ZAKR)
#define soilTempPin 32      // Датчик температуры почвы (освобожден UPR_1)
#define btnModePin 33       // Кнопка выбора режима (освобожден UPR_2)
#define btnConfirmPin 25    // Кнопка подтверждения (освобожден UPR_3)
#define gruntPin 4          // ADC датчик влажности почвы
#define krushPin 19         // Цифровой датчик влажности почвы (сухо/мокро)
#define rcPin 16            // RF передатчик
#define relayPhylampPin 26  // Реле фитолампы на плате ESP32
#define relayFillPin 2      // Реле клапана налива (внешнее реле или на плате)

// =================== TCA9555 НАСТРОЙКИ ===================
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

// =================== ОБЪЕКТЫ ===================
GyverHub hub("MyDevices", "Ogorod_esp32", "");
PairsFile data(&GH_FS, "/data.dat", 3000);
FastBot2 bot;
DHT dht(DHTPIN, DHTTYPE);
LCD_1602_RUS lcd(0x27, 16, 2);
FlowSensor SensorPoliv(YFS201, manPin);
FlowSensor SensorNaliv(YFS201, fillPin);
RCSwitch mySwitch = RCSwitch();
GyverNTP ntp(3);

// =================== ПЕРЕМЕННЫЕ ===================
long duration, cm;
float t, h, d_poliv, d_naliv;
float soilTemp = 0;
int g;                      // ADC влажность почвы
bool k = 0;                 // krushPin - влажность почвы (1=сухо, 0=мокро)
int j = 0;                  // Выбор режима полива (0-5)
int y_greenhouse = 6;       // Часы старта - теплица
int y_drip = 7;             // Часы старта - капельный
int y_sprinkler = 8;        // Часы старта - спринклер
int y_phylamp = 18;         // Часы старта - фитолампа
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

// Состояния режимов
bool mode_greenhouse = false;   // Режим теплицы
bool mode_drip = false;         // Режим капельный
bool mode_sprinkler = false;    // Режим спринклерный
bool mode_fill = false;         // Режим автоналива
bool mode_phylamp = false;      // Режим фитолампы
bool mode_framuga = false;      // Режим фрамуги

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

// Состояние LCD
enum LCDState {
  LCD_TIME,
  LCD_LEVEL,
  LCD_MODE_SELECT,
  LCD_ACTIVE_MODE
};
LCDState lcdState = LCD_TIME;
unsigned long lcdTimer = 0;
const unsigned long LCD_TIMEOUT = 10000;  // 10 сек возврата к времени

// GyverHub переменные
static byte tab;
static byte sel;
int o = 0;  // счетчик для телеграмма

// =================== TCA9555 ФУНКЦИИ ===================
void TCA9555_init() {
  Wire.beginTransmission(TCA9555_ADDR);
  Wire.write(0x06);  // Configuration register 0
  Wire.write(0x00);  // Все пины на выход
  Wire.write(0x00);  // Configuration register 1
  Wire.endTransmission();
}

void TCA9555_write(uint16_t value) {
  Wire.beginTransmission(TCA9555_ADDR);
  Wire.write(0x02);  // Output register 0
  Wire.write(value & 0xFF);       // Port 0
  Wire.write((value >> 8) & 0xFF); // Port 1
  Wire.endTransmission();
}

void TCA9555_setRelay(byte relay, bool state) {
  static uint16_t relayState = 0;
  if (state) {
    relayState |= (1 << relay);
  } else {
    relayState &= ~(1 << relay);
  }
  TCA9555_write(relayState);
}

void TCA9555_allOff() {
  TCA9555_write(0x0000);
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
      // Долгое нажатие - вход в меню выбора режимов
      if (lcdState != LCD_MODE_SELECT) {
        lcdState = LCD_MODE_SELECT;
        lcdTimer = millis();
      }
      btnModePressTime = millis();  // Сброс для следующего нажатия
    }
  } else {
    if (btnModePressTime > 0 && millis() - btnModePressTime < LONG_PRESS_TIME) {
      // Короткое нажатие - переключение режима
      if (lcdState == LCD_MODE_SELECT) {
        j = (j + 1) % 6;
        lcdTimer = millis();
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
      }
      btnConfirmPressTime = millis();
    }
  } else {
    if (btnConfirmPressTime > 0 && millis() - btnConfirmPressTime < LONG_PRESS_TIME) {
      // Короткое нажатие - подтверждение выбора режима
      if (lcdState == LCD_MODE_SELECT) {
        // Подтверждение выбора
        lcdState = LCD_TIME;
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
      }
    }
    btnConfirmPressTime = 0;
  }

  // Таймаут возврата к времени
  if (lcdState == LCD_MODE_SELECT && millis() - lcdTimer > LCD_TIMEOUT) {
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

// =================== GyverHub BUILD ===================
void build(gh::Builder& b) {
  if (b.beginRow()) {
    b.Label("дата ").label(F("ВРЕМЯ")).size(2, 1).fontSize(25).value(stamp_1);
    b.Label("время ").label(F("ДАТА")).size(2, 1).fontSize(25).value(stamp_2);
    b.endRow();
  }

  {
    gh::Row r(b);
    b.Display_("disp_1", val).label(F("КРЫША")).size(3);
    b.Display_("disp_2", val_1).label(F("ПОЧВА")).size(3);
    b.Label_(F("disp_3")).label(F("t почвы")).size(2).fontSize(20).value(soilTemp, 1);
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

  {
    gh::Row r(b);
    b.Label_(F("tim_gh")).label(F("таймеры")).size(2).fontSize(10).value("настр");
    b.Input_("inp_gh", &y_greenhouse).label(F("теплица ч")).size(1).fontSize(10).value(y_greenhouse);
    b.Input_("inp_gh2", &y_drip).label(F("капель ч")).size(1).fontSize(10).value(y_drip);
    b.Input_("inp_gh3", &y_sprinkler).label(F("сприн ч")).size(1).fontSize(10).value(y_sprinkler);
    b.Input_("inp_gh4", &y_phylamp).label(F("фито ч")).size(1).fontSize(10).value(y_phylamp);
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
    cv0.stroke(0xff0000);
    cv0.strokeWeight(5);
    cv0.fill(gh::Color(0, 0, 255, 0));
    cv0.background();
    cv0.line(0, 0, 100, 0);
    cv0.line(0, 0, 0, 100);
    cv0.line(0, 100, 100, 100);
    cv0.line(100, 100, 100, 0);
    b.EndCanvas();
    b.endRow();
  }
}

// =================== РЕЖИМЫ РАБОТЫ ===================

// Режим 0: Полив теплицы
void runGreenhouse() {
  if (!mode_greenhouse) return;

  // Запрет при влажности почвы k==1 (сухо)
  if (k == 1) return;

  // Запрет при низком уровне бака
  if (cm < 18) return;

  static unsigned long processStart = 0;
  static byte step = 0;

  if (ntp.hour() == y_greenhouse && ntp.minute() == 0 && step == 0) {
    step = 1;
    processStart = millis();
    // Включаем клапан теплицы и фазу
    TCA9555_setRelay(TCA_RELAY_GREENHOUSE, true);
    TCA9555_setRelay(TCA_RELAY_PHASE, true);
    flag_greenhouse = true;
    flag_phase = true;
    lcdState = LCD_ACTIVE_MODE;
    lcd.setCursor(0, 0);
    lcd.print("полив теплицы ");
  }

  if (step == 1) {
    // Через 10 сек выключаем фазу
    if (millis() - processStart >= 10000) {
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      flag_phase = false;
      processStart = millis();
      step = 2;
    }
  }

  if (step == 2) {
    // Работаем 30 минут
    if (millis() - processStart >= (unsigned long)duration_greenhouse * 60000) {
      TCA9555_setRelay(TCA_RELAY_GREENHOUSE, false);
      flag_greenhouse = false;
      step = 0;
      lcdState = LCD_TIME;
      bot.sendMessage(fb::Message("полив теплицы завершен", CHAT_ID));
    }
  }
}

// Режим 1: Капельный полив
void runDrip() {
  if (!mode_drip) return;

  // Запрет при влажности почвы k==1 (сухо)
  if (k == 1) return;

  // Запрет при низком уровне бака
  if (cm < 18) return;

  static unsigned long processStart = 0;
  static byte step = 0;

  if (ntp.hour() == y_drip && ntp.minute() == 0 && step == 0) {
    step = 1;
    processStart = millis();
    // Включаем клапан капельного и фазу на 10 сек
    TCA9555_setRelay(TCA_RELAY_DRIP, true);
    TCA9555_setRelay(TCA_RELAY_PHASE, true);
    flag_drip = true;
    flag_phase = true;
    lcdState = LCD_ACTIVE_MODE;
    lcd.setCursor(0, 0);
    lcd.print("капельный полив");
  }

  if (step == 1) {
    // Через 10 сек выключаем фазу и клапан, включаем насос
    if (millis() - processStart >= 10000) {
      TCA9555_setRelay(TCA_RELAY_DRIP, false);
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      TCA9555_setRelay(TCA_RELAY_PUMP, true);
      flag_drip = false;
      flag_phase = false;
      flag_pump = true;
      pumpStartTime = millis();
      flowCheckTime = millis();
      flowDetected = false;
      processStart = millis();
      step = 2;
    }
  }

  if (step == 2) {
    // Проверка потока через 3 сек после включения насоса
    if (millis() - flowCheckTime >= 3000 && !flowDetected) {
      if (d_poliv < 0.5) {  // Нет потока
        TCA9555_setRelay(TCA_RELAY_PUMP, false);
        flag_pump = false;
        step = 0;
        lcdState = LCD_TIME;
        hub.update("ld_alarm_flow").value(1);
        bot.sendMessage(fb::Message("нет потока! насос выкл", CHAT_ID));
        return;
      }
      flowDetected = true;
    }

    // Насос работает 10 минут
    if (millis() - pumpStartTime >= 10 * 60000) {
      TCA9555_setRelay(TCA_RELAY_PUMP, false);
      flag_pump = false;
      // Включаем фазу на 10 сек
      TCA9555_setRelay(TCA_RELAY_PHASE, true);
      flag_phase = true;
      phaseStartTime = millis();
      step = 3;
    }
  }

  if (step == 3) {
    // Через 10 сек выключаем фазу
    if (millis() - phaseStartTime >= 10000) {
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      flag_phase = false;
      step = 0;
      lcdState = LCD_TIME;
      bot.sendMessage(fb::Message("капельный полив завершен", CHAT_ID));
    }
  }
}

// Режим 2: Спринклерный полив (3 направления)
void runSprinkler() {
  if (!mode_sprinkler) return;

  // Запрет при влажности почвы k==1 (сухо)
  if (k == 1) return;

  // Запрет при низком уровне бака
  if (cm < 18) return;

  static unsigned long processStart = 0;
  static byte step = 0;
  static byte direction = 0;

  if (ntp.hour() == y_sprinkler && ntp.minute() == 0 && step == 0) {
    step = 1;
    direction = 1;
    processStart = millis();
  }

  // Направление 1
  if (step == 1 && direction == 1) {
    if (millis() - processStart >= 0 && !flag_dir1) {
      TCA9555_setRelay(TCA_RELAY_DIR1, true);
      TCA9555_setRelay(TCA_RELAY_PHASE, true);
      flag_dir1 = true;
      flag_phase = true;
      dirStartTime = millis();
      lcdState = LCD_ACTIVE_MODE;
      lcd.setCursor(0, 0);
      lcd.print("спринклер 1 напр");
    }
    if (millis() - dirStartTime >= 10000) {
      TCA9555_setRelay(TCA_RELAY_DIR1, false);
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      TCA9555_setRelay(TCA_RELAY_PUMP, true);
      flag_dir1 = false;
      flag_phase = false;
      flag_pump = true;
      pumpStartTime = millis();
      flowCheckTime = millis();
      flowDetected = false;
      step = 2;
    }
  }

  if (step == 2 && direction == 1) {
    // Проверка потока
    if (millis() - flowCheckTime >= 3000 && !flowDetected) {
      if (d_poliv < 0.5) {
        TCA9555_setRelay(TCA_RELAY_PUMP, false);
        flag_pump = false;
        step = 0;
        lcdState = LCD_TIME;
        hub.update("ld_alarm_flow").value(1);
        bot.sendMessage(fb::Message("нет потока! насос выкл", CHAT_ID));
        return;
      }
      flowDetected = true;
    }

    if (millis() - pumpStartTime >= 5 * 60000) {
      TCA9555_setRelay(TCA_RELAY_PUMP, false);
      flag_pump = false;
      TCA9555_setRelay(TCA_RELAY_PHASE, true);
      flag_phase = true;
      phaseStartTime = millis();
      step = 3;
    }
  }

  if (step == 3 && direction == 1) {
    if (millis() - phaseStartTime >= 10000) {
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      flag_phase = false;
      direction = 2;
      step = 4;
      processStart = millis();
    }
  }

  // Направление 2
  if (step == 4 && direction == 2) {
    if (millis() - processStart >= 0 && !flag_dir2) {
      TCA9555_setRelay(TCA_RELAY_DIR2, true);
      TCA9555_setRelay(TCA_RELAY_PHASE, true);
      flag_dir2 = true;
      flag_phase = true;
      dirStartTime = millis();
      lcd.setCursor(0, 0);
      lcd.print("спринклер 2 напр");
    }
    if (millis() - dirStartTime >= 10000) {
      TCA9555_setRelay(TCA_RELAY_DIR2, false);
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      TCA9555_setRelay(TCA_RELAY_PUMP, true);
      flag_dir2 = false;
      flag_phase = false;
      flag_pump = true;
      pumpStartTime = millis();
      flowCheckTime = millis();
      flowDetected = false;
      step = 5;
    }
  }

  if (step == 5 && direction == 2) {
    if (millis() - flowCheckTime >= 3000 && !flowDetected) {
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

    if (millis() - pumpStartTime >= 5 * 60000) {
      TCA9555_setRelay(TCA_RELAY_PUMP, false);
      flag_pump = false;
      TCA9555_setRelay(TCA_RELAY_PHASE, true);
      flag_phase = true;
      phaseStartTime = millis();
      step = 6;
    }
  }

  if (step == 6 && direction == 2) {
    if (millis() - phaseStartTime >= 10000) {
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      flag_phase = false;
      direction = 3;
      step = 7;
      processStart = millis();
    }
  }

  // Направление 3
  if (step == 7 && direction == 3) {
    if (millis() - processStart >= 0 && !flag_dir3) {
      TCA9555_setRelay(TCA_RELAY_DIR3, true);
      TCA9555_setRelay(TCA_RELAY_PHASE, true);
      flag_dir3 = true;
      flag_phase = true;
      dirStartTime = millis();
      lcd.setCursor(0, 0);
      lcd.print("спринклер 3 напр");
    }
    if (millis() - dirStartTime >= 10000) {
      TCA9555_setRelay(TCA_RELAY_DIR3, false);
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      TCA9555_setRelay(TCA_RELAY_PUMP, true);
      flag_dir3 = false;
      flag_phase = false;
      flag_pump = true;
      pumpStartTime = millis();
      flowCheckTime = millis();
      flowDetected = false;
      step = 8;
    }
  }

  if (step == 8 && direction == 3) {
    if (millis() - flowCheckTime >= 3000 && !flowDetected) {
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

    if (millis() - pumpStartTime >= 5 * 60000) {
      TCA9555_setRelay(TCA_RELAY_PUMP, false);
      flag_pump = false;
      TCA9555_setRelay(TCA_RELAY_PHASE, true);
      flag_phase = true;
      phaseStartTime = millis();
      step = 9;
    }
  }

  if (step == 9 && direction == 3) {
    if (millis() - phaseStartTime >= 10000) {
      TCA9555_setRelay(TCA_RELAY_PHASE, false);
      flag_phase = false;
      step = 0;
      lcdState = LCD_TIME;
      bot.sendMessage(fb::Message("спринклерный полив завершен", CHAT_ID));
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
    lcd.setCursor(0, 0);
    lcd.print("автоналив       ");
    bot.sendMessage(fb::Message("начат автоналив", CHAT_ID));
  }

  // Уровень достиг 80 см (100-cm >= 80, т.е. cm <= 20)
  if (filling && cm <= 20) {
    filling = false;
    digitalWrite(relayFillPin, LOW);
    flag_fill = false;
    lcdState = LCD_TIME;
    bot.sendMessage(fb::Message("автоналив завершен", CHAT_ID));
  }
}

// Режим 4: Фитолампа
void runPhylamp() {
  if (!mode_phylamp) return;

  static unsigned long processStart = 0;
  static bool running = false;

  if (ntp.hour() == y_phylamp && ntp.minute() == 0 && !running) {
    running = true;
    processStart = millis();
    // Включаем фитолампу
    digitalWrite(relayPhylampPin, HIGH);
    flag_phylamp = true;
    lcdState = LCD_ACTIVE_MODE;
    lcd.setCursor(0, 0);
    lcd.print("фитолампа       ");
  }

  if (running) {
    // Работаем 4 часа
    if (millis() - processStart >= 4 * 3600000UL) {
      digitalWrite(relayPhylampPin, LOW);
      flag_phylamp = false;
      running = false;
      lcdState = LCD_TIME;
      bot.sendMessage(fb::Message("фитолампа выключена", CHAT_ID));
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
    lcd.setCursor(0, 0);
    lcd.print("фрамуга откр    ");
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
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("УРОВЕНЬ ");
  lcd.print(100 - cm);
  lcd.print("%  ");

  // Горизонтальная шкала в нижнем ряду
  lcd.setCursor(0, 1);
  lcd.print("[");
  int barWidth = map(constrain(100 - cm, 0, 100), 0, 100, 0, 14);
  for (int i = 0; i < 14; i++) {
    if (i < barWidth) {
      lcd.print("#");
    } else {
      lcd.print(" ");
    }
  }
  lcd.print("]");
}

void lcdShowTime() {
  lcd.setCursor(0, 0);
  lcd.print(stamp_2);
  lcd.setCursor(0, 1);
  lcd.print(stamp_1);
}

void lcdShowModeSelect() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ВЫБОР РЕЖИМА:");
  lcd.setCursor(0, 1);
  switch (j) {
    case 0: lcd.print("теплица  "); break;
    case 1: lcd.print("капельный"); break;
    case 2: lcd.print("спринклер"); break;
    case 3: lcd.print("автоналив"); break;
    case 4: lcd.print("фитолампа"); break;
    case 5: lcd.print("фрамуга  "); break;
  }
  lcd.print(mode_greenhouse || mode_drip || mode_sprinkler || mode_fill || mode_phylamp || mode_framuga ? " вкл" : " выкл");
}

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);

  // Инициализация пинов
  pinMode(manPin, INPUT_PULLUP);
  pinMode(fillPin, INPUT_PULLUP);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(krushPin, INPUT);
  pinMode(gruntPin, INPUT);
  pinMode(btnModePin, INPUT_PULLUP);
  pinMode(btnConfirmPin, INPUT_PULLUP);
  pinMode(relayPhylampPin, OUTPUT);
  pinMode(relayFillPin, OUTPUT);

  digitalWrite(relayPhylampPin, LOW);
  digitalWrite(relayFillPin, LOW);

  // Прерывания кнопок
  attachInterrupt(digitalPinToInterrupt(btnModePin), btnModeISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(btnConfirmPin), btnConfirmISR, FALLING);

  // Инициализация I2C и TCA9555
  Wire.begin();
  TCA9555_init();
  TCA9555_allOff();

  // Датчики
  dht.begin();
  SensorPoliv.begin(countPoliv);
  SensorNaliv.begin(countNaliv);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.print("Огород v2.0");
  delay(2000);

  // WiFi и сеть
#ifdef GH_ESP_BUILD
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println(WiFi.localIP());
  ntp.begin();

  hub.mqtt.config("m9.wqtt.ru", 20042, "u_F84LY5", "rdu4PozQ");
  hub.setVersion("AndyInjiner/Ogorod@2.0");
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
  bot.attachUpdate(updateh);
  bot.setToken(F(BOT_TOKEN));
  bot.setPollMode(fb::Poll::Long, 20000);
  bot.setLimit(1);

  fb::Message msg("Меню управления", CHAT_ID);
  fb::InlineMenu menu("restart;status;fill\nstop_fill;test", "n_1;n_2;n_3;n_4;n_5");
  msg.setInlineMenu(menu);
  bot.sendMessage(msg);
  bot.sendMessage(fb::Message("система запущена", CHAT_ID));

  hub.begin();
  data.begin();
}

// =================== ОБРАБОТЧИК БОТА ===================
void updateh(fb::Update& u) {
  if (u.isQuery()) {
    Serial.println("NEW QUERY");
    Serial.println(u.query().data());

    bot.answerCallbackQuery(u.query().id(), "query answered");

    switch (u.query().data().hash()) {
      case "n_1"_h:
        ESP.restart();
        break;
      case "n_2"_h:
        bot.sendMessage(fb::Message("статус: система работает", CHAT_ID));
        break;
      case "n_3"_h:
        mode_fill = true;
        hub.sendRefresh();
        break;
      case "n_4"_h:
        mode_fill = false;
        hub.sendRefresh();
        break;
      case "n_5"_h:
        // Тест реле
        TCA9555_setRelay(TCA_RELAY_PUMP, true);
        delay(1000);
        TCA9555_setRelay(TCA_RELAY_PUMP, false);
        break;
    }
  }
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
  duration = pulseIn(PIN_ECHO, HIGH);
  cm = (duration / 2) / 29.1;

  // Датчик потока полив
  SensorPoliv.read();
  d_poliv = SensorPoliv.getFlowRate_m();

  // Датчик потока налив
  SensorNaliv.read();
  d_naliv = SensorNaliv.getFlowRate_m();

  // Датчик влажности почвы (цифровой)
  k = digitalRead(krushPin);
  if (k == 1) {
    val = s_2;  // сухая
  } else {
    val = s_1;  // дождь (влажная)
  }

  // Датчик влажности почвы (ADC)
  g = analogRead(gruntPin);
  if (g < 300) {
    val_1 = s_5;  // мокрая
  } else if (g > 350 && g < 750) {
    val_1 = s_4;  // влажная
  } else if (g > 800) {
    val_1 = s_3;  // высохла
  }

  // Датчик температуры почвы (DS18B20 или аналогичный)
  // Пока заглушка - нужно подключить реальный датчик
  soilTemp = t;  // Временно используем температуру воздуха
}

// =================== LOOP ===================
void loop() {
  hub.tick();
  ntp.tick();
  data.tick();
  bot.tick();

  // Чтение датчиков
  readSensors();

  // Проверка кнопок
  checkButtons();

  // Обновление LCD
  switch (lcdState) {
    case LCD_TIME:
      lcdShowTime();
      break;
    case LCD_LEVEL:
      lcdDrawLevelBar();
      break;
    case LCD_MODE_SELECT:
      lcdShowModeSelect();
      if (millis() - lcdTimer > LCD_TIMEOUT) {
        lcdState = LCD_TIME;
      }
      break;
    case LCD_ACTIVE_MODE:
      // Остается показывать активный режим
      break;
  }

  // Режимы работы
  runGreenhouse();
  runDrip();
  runSprinkler();
  runAutoFill();
  runPhylamp();
  runFramuga();

  // Таймеры обновлений
  static gh::Timer tmr(3000);
  static gh::Timer tmr_1(1000);
  static gh::Timer tmr_2(20000);
  static gh::Timer tmr_3(50000);

  if (tmr) {
    hub.update(F("tempr")).value(t);
    hub.update(F("humid")).value(h);
    hub.update(F("tank")).value(100 - cm);
    hub.update(F("disp_3")).value(soilTemp, 1);
    hub.update(F("pot_poliv")).value(d_poliv, 2);
    hub.update(F("pot_naliv")).value(d_naliv, 2);
    hub.update(F("disp_1")).value(val);
    hub.update(F("disp_2")).value(val_1);

    // Canvas уровень бака
    gh::CanvasUpdate cv0("cv0", &hub);
    cv0.noStroke();
    cv0.clearRect(0, 0, 100, 100 - (100 - cm));
    cv0.fill(gh::Color(0, 255, 0, 0));
    cv0.rect(0, 0, 100, 100 - (100 - cm));
    cv0.send();

    // LED состояния режимов
    hub.update("ld_greenhouse").value(mode_greenhouse);
    hub.update("ld_drip").value(mode_drip);
    hub.update("ld_sprinkler").value(mode_sprinkler);
    hub.update("ld_fill").value(mode_fill);
    hub.update("ld_phylamp").value(mode_phylamp);
    hub.update("ld_framuga").value(mode_framuga);

    // Аварийный уровень
    if (cm < 18) {
      hub.update("ld_alarm_level").value(1);
    } else {
      hub.update("ld_alarm_level").value(0);
    }
  }

  if (tmr_1) {
    stamp_1 = ntp.timeToString();
    stamp_2 = ntp.dateToString();
    hub.update("date").value(stamp_1);
    hub.update("time").value(stamp_2);
  }

  if (tmr_2) {
    // Аварийное сообщение при низком уровне
    if (cm < 18) {
      bot.sendMessage(fb::Message("бак пустой! уровень критический", CHAT_ID));
    }
  }

  if (tmr_3) {
    hub.sendRefresh();
  }
}
