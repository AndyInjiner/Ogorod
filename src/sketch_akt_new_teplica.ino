#include <DHT.h>
#include <GyverNTP.h>
//#include <AutoOTA.h>
// базовый пример работы с библиотекой, основные возможности. Загрузи и изучай код

// логин-пароль от роутера
#define AP_SSID "MERCUSYS_3BCA"
#define AP_PASS "router320"

#include <Arduino.h>
#include <GyverHub.h>
 //#define GH_NO_OTA      // отключить ОТА обновления (для esp)
 //#define GH_NO_OTA_URL  // отключить ОТА по ссылке (для esp)

//GyverHub hub;
//hub.setVersion("AndyInjiner/Ogorod@1.0");
 GyverHub hub("MyDevices", "Ogorod", "");  // можно настроить тут, но без F-строк!
 #include <PairsFile.h>
 PairsFile data(&GH_FS, "/data.dat", 3000);
#define DHTPIN D4      // тут датчик темп и влаги What digital pin we're connected to
#define DHTTYPE DHT11  //  обьяв типа температура и влаж DHT 11<p>DHT dht(DHTPIN, DHTTYPE);
#define PIN_TRIG D1 // уровень в баке
#define PIN_ECHO D2 // уровень в баке
#define dataPin  D6 //Пин подключен к DS входу 74HC595
#define latchPin D7 //Пин подключен к ST_CP входу 74HC595
#define clockPin  D8 //Пин подключен к SH_CP входу 74HC595
#define manPin D5 //авар останов , будет датчик расхода
#define gruntPin A0 // почва сухо
long duration, cm;
GyverNTP ntp(3);
// обработчик кнопки (см. ниже)
//pressure_sensor.begin(DOUT_Pin, SCLK_Pin);
DHT dht(DHTPIN, DHTTYPE);
float t;
float h;
unsigned long d ;
byte data_1 ;
int a=5;
bool s=0;
static byte tab;
static byte sel;
String  stamp_1;
String  stamp_2;
bool flag_1 = 0;
bool flag_2 = 0;
bool flag_3 = 0;
bool flag_4 = 0;
bool flag_5 = 0;
bool dsbl, nolbl, notab;
int p; // расходомер
float g; // почва
int j ;//выбор режима полива
  String s_1 = data["key0"];
  String s_2 = data["key1"];
  String s_3 = data["key2"]; 
  String s_4 = data["key3"];
  String s_5 = data["key4"];
void btn_ca() {
  Serial.println("click 1");
  a=1;
  flag_1 = 1;

}
void btn_cb() {
  Serial.println("click 2");
   a=2;
  flag_2 = 1;
}
void btn_cc() {
  Serial.println("click 3");
   a=3;
  flag_3 = 1;
}
void btn_cd() {
  Serial.println("click 4");
   a=4;
  flag_4 = 1;
}
void btn_cf() {
  Serial.println("click 5");
  flag_5 = 1;
}
void btn_ca_1() {
  Serial.println("click 1");
  a=5;
  flag_1 = 0;
}
void btn_cb_1() {
  Serial.println("click 2");
  a=5;
  flag_2 = 0;
}
void btn_cc_1() {
  Serial.println("click 3");
  a=5;
  flag_3 = 0;
}
void btn_cd_1() {
  Serial.println("click 4");
  a=5;
  flag_4 = 0;
}
void btn_cf_1() {
  Serial.println("click 5");
  flag_5 = 0;
}
void build(gh::Builder& b) {
   if (b.beginRow()) {
     b.Label("дата ").size(2,1).fontSize(25).value(stamp_1);
    b.Label("время ").size(2,1).fontSize(25).value(stamp_2);
    // b.DateTime_("datetime").value(stamp);
    //hub.update(F("main")).Time(ntp.timeString()).Date(ntp.dateString());
    b.endRow();
  }
  {
    gh::Row r(b);
    b.Text_("disp_1",s_1);
    b.Text_("disp_2",s_2 );
     
  }
  {
  gh::Row r(b);
  if (b.Tabs(&tab).text("пуск;стоп").click()) b.refresh(); // обновить по клику
  switch (tab) {
    case 0:
         Serial.println("пуск");
         s=1;
        break;
    case 1:
        Serial.println("стоп") ;
        s=0;
        break;
  }
   if(b.Select(&sel).text("полив;насос;полив и насос").click()) b.refresh();
   switch (sel) {
    case 0:
         Serial.println("полив");
         j=1;
        break;
    case 1:
        Serial.println("насос") ;
        j=2;
        break;
    case 2:
        Serial.println("полив и насос");
        j=3;
        break;
  }
  }
  {
    gh::Row r(b);
     //b.Label("таймер ").size(2,1).value("off");
     b.Label_(F("tim")).label(F("таймер")).size(2).fontSize(20).value("выкл");
     b.Label_(F("pot")).label(F("поток")).size(2).fontSize(20).value("0");
     b.LED_("ld5").label(F("авар останов")).color(gh::Colors::Red).value(0);
  }  // b.LED().value(1); =======
   if (b.beginRow()) {
     b.Button().attach(btn_ca).label(F("насос вкл")).color(gh::Colors::Green);
    b.Button().attach(btn_cb).label(F("полив открыть")).color(gh::Colors::Green);
    b.Button().attach(btn_cc).label(F("вентилятор вкл")).color(gh::Colors::Green);
    b.Button().attach(btn_cd).label(F("окна открыть ")).color(gh::Colors::Green);

    b.endRow();
  }
  if (b.beginRow()) {
     b.LED_("ld1").value(0);
     b.LED_("ld2").value(0);
     b.LED_("ld3").value(0);
     b.LED_("ld4").value(0);

    b.endRow();
  }
  if (b.beginRow()) {

    b.Button().attach(btn_ca_1).label(F("насос выкл")).color(gh::Colors::Red);
    b.Button().attach(btn_cb_1).label(F("полив закрыть")).color(gh::Colors::Red);
    b.Button().attach(btn_cc_1).label(F("вентилятор выкл")).color(gh::Colors::Red);
    b.Button().attach(btn_cd_1).label(F("окна закрыть")).color(gh::Colors::Red);

    b.endRow();
  }
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
    b.Label("вода  в баке");
     b.endRow();
  }
  if (b.beginRow()) {
    gh::Canvas cv0;
    b.BeginCanvas_(F("cv0"), 100, 100,&cv0);
    cv0.stroke(0xff0000);
    cv0.strokeWeight(5);
    //cv0.fill(gh::Color(0, 0, 255, 0));
    //cv0.background();
    cv0.line(0, 0, 100, 0);
    cv0.line(0, 0, 0, 100);
    cv0.line(0, 100, 100, 100);
    cv0.line(100, 100, 100, 0);
    b.EndCanvas();



    // =================== ИНФО О БИЛДЕ ===================
    b.endRow();
  }
  }
  //AutoOTA ota("0.1", "AndyInjiner/Ogorod/main/project.json");

void setup() {
  Serial.begin(115200);
  pinMode(manPin, INPUT_PULLUP);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(latchPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(dataPin, OUTPUT);
  dht.begin();
#ifdef GH_ESP_BUILD
  // подключение к роутеру
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println(WiFi.localIP());
  ntp.begin();
  // если нужен MQTT - подключаемся
  hub.mqtt.config("test.mosquitto.org", 1883);
  // hub.mqtt.config("test.mosquitto.org", 1883, "login", "pass");
  hub.setVersion("AndyInjiner/Ogorod@0.1");
  // ИЛИ

  // режим точки доступа
  // WiFi.mode(WIFI_AP);
  // WiFi.softAP("My Hub");
  // Serial.println(WiFi.softAPIP());    // по умолч. 192.168.4.1
#endif
  // указать префикс сети, имя устройства и иконку
  data["key0"] = "дождь";
  data["key1"] = "сухо";
  data["key2"] = "почва высохла";
  data["key3"] = "почва влажная";
  data["key4"] = "почва мокрая";
  //String s_1 = data["key0"];
  //String s_2 = data["key1"];
  //String s_3 = data["key2"]; 
  //String s_4 = data["key3"];
  //String s_5 = data["key4"];

  hub.config(F("MyDevices"), F("OGOROD"), F(""));

  // подключить билдер
  hub.onBuild(build);

  // запуск!
  ntp.begin();
  hub.begin();
  data.begin();

  //AutoOTA ota("0.1", "AndyInjiner/Ogorod/main/project.json");
  //if (ota.checkUpdate()) {
         //ota.updateNow();
  //a=5;

}
 //String ver, notes;
//if (ota.checkUpdate(&ver, &notes)) {
    //Serial.println(ver);
    //Serial.println(notes);
//}

 void reg(byte data_1) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, LSBFIRST, data_1);
  digitalWrite(latchPin, HIGH);
   Serial.println("вкл");
}
void str(){
  if (j==2||j==3){
    if (ntp.minute() >=10 & ntp.minute() <=19) {
    hub.update(F("tim")).value("насос ON"); 
    //hub.update("таймер").value("on");
    flag_1=1;
    a=1;
     Serial.println("старт");}
    if (ntp.minute() == 20) {
    hub.update(F("tim")).label(F("таймер")).value("выкл"); 
    //hub.update("таймер").value("on");
    flag_1=0;
    a=5;
     Serial.println("стоп"); 
  }
}
}
  void str_1(){
    if (j==1||j==3){
     if (ntp.minute() >=22 & ntp.minute() <=33) {
     hub.update(F("tim")).value("полив ON"); 
     //hub.update("таймер").value("on");
     flag_2=1;
     a=2;
     Serial.println("старт");}
    if (ntp.minute() == 35) {
    hub.update(F("tim")).label(F("таймер")).value("выкл"); 
    //hub.update("таймер").value("on");
    flag_2=0;
    a=5;
     Serial.println("стоп"); 
  }
    }
}
void loop() {
  // =================== ТИКЕР ===================
  // вызываем тикер в главном цикле программы
  // он обеспечивает работу связи, таймаутов и прочего
  hub.tick();
  ntp.tick();
 float h = dht.readHumidity();
 float t = dht.readTemperature();
  //r = digitalRead(manPin);
  g = analogRead(gruntPin);
  // =========== ОБНОВЛЕНИЯ ПО ТАЙМЕРУ ===========
  // в библиотеке предусмотрен удобный класс асинхронного таймера
  static gh::Timer tmr(3000);  // период 1 секунда
  static gh::Timer tmr_1(1000);
   static gh::Timer tmr_2(3000);
   static gh::Timer tmr_3(50000);
     // период 1 секунда
  // каждую секунду будем обновлять заголовок
  if (tmr) {
    hub.update(F("tempr")).value(t);
    hub.update(F("humid")).value(h);
   //hub.sendUpdate("Time",stamp);
    gh::CanvasUpdate cv0("cv0", &hub);
    //cv0.translate(0, cm);
    cv0.noStroke();
    cv0.clearRect(0,0,100,100);
    cv0.fill(gh::Color(0, 255, 0, 0));
    cv0.rect(0,0,100,100-(100-cm));
    //cv0.fill(gh::Color(0, 0, 0, 255));
    cv0.send();
    digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  duration = pulseIn(PIN_ECHO, HIGH);
  cm = (duration / 2) / 29.1;
    hub.update("ld1").value(flag_1);
    hub.update("ld2").value(flag_2);
    hub.update("ld3").value(flag_3);
    hub.update("ld4").value(flag_4);
    //hub.update("ld6").value(flag_6);
     if (s==1) {
      str();
      str_1();}
       //Serial.println(s);
}
  if (tmr_1){
    Serial.println(ntp.timeString());
    Serial.println(ntp.dateString());
     stamp_1=ntp.timeString() ;
     stamp_2=ntp.dateString() ;
    hub.update("date").value(stamp_1);
    hub.update("time").value(stamp_2);
    if(digitalRead(manPin)){
    p++;}
    if(p>=100){
      p=0;
    }
     hub.update(F("pot")).value(p);  
  }
   if(tmr_2){
    if (cm >= 90) {
  a=5;
  hub.update("ld5").label(F("авар останов")).value(1);
}
   switch(a){
    case 1:reg(B11111011) ;break;
    case 2:reg(B11011111) ;break;
    case 3:reg(B11101111) ;break;
    case 4:reg(B11110111) ;break;
    case 5:reg(B11111111) ;break;

    //case 3: Serial.println("3");break;
}
  //hub.sendRefresh();
 Serial.println(g);
   }
   if(tmr_3){
     data.tick();
     hub.update(F("disp_1")).value(s_1);
     hub.update(F("disp_2")).value(s_2);
    hub.sendRefresh();
   }

}