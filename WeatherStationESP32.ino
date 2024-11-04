// Датчик SHT41, SHT31  SDA - 21, SCL - 22
// Экран SSH1106 1,3''  SDA - 21, SCL - 22
// Esp32 с антенной 
//***********************************************************************************************
// Константы и дефайны
//***********************************************************************************************
const char* ssid = "*****";          // WiFi login 
const char* password = "*****";      // WiFi password
const char* mqttLogin = "*******";   // mqtt Login
const char* mqttPass = "********";   // mqtt Password

const char* hubPrefix = "********";  // GyverHub hubPrefix
const char* hubClientID = "******";  // GyverHub Client ID
const char* OpenMonKey = "*******";  // Open Monitoring Key
const char* otaPass = "*********";        // OTA Password

//Необходимо выбрать, какой используется датчик температуры и влажности и оставить только одну строку. Другие строки должны быть закомментированы.
#define USE_SHT41                           //использовать датчик Sensirion SHT41
//#define USE_SHT31                           //использовать датчик Sensirion SHT31

#ifdef USE_SHT31                    // если используется датчик SHT31
#define heat3xTime 5*60*1000L       // время, на которое включается нагрев датчика SHT3x
#include <SensirionI2cSht3x.h>      // библиотека датчиков температуры и влажности SHT3х
SensirionI2cSht3x sht3x;            // создание объекта датчика sht3x библиотеки SensirionI2cSht3x
#endif

#ifdef USE_SHT41                   // если используется датчик SHT41
#define heat4xBorder 75            // значение влажности, выше которого включается нагрев датчика SHT4x 
#include <SensirionI2cSht4x.h>     // библиотека датчиков температуры и влажности SHT4х
SensirionI2cSht4x sht4x;           // создание объекта датчика sht4x библиотеки SensirionI2cSht4x
#endif

#define sensorReadPeriod 1000      // период между опросом датчика в мс.
#define openMonPeriod 5*60*1000L   // период между отправкой данных на сервер ОМ в мс
#define narodMonPeriod 10*60*1000L // период между отправкой данных на сервер NM в мс
#define checkWifiPeriod 30*1000L   // период проверки состояния WiFi соединения в мс
#define heat4xPeriod 120*1000L     // период включения нагрева SHT4x 
#define heatPeriod 24*60*60*1000L  // период безусловного включения нагрева любого датчика  (время МЕЖДУ включениями)
#define oledInvertPeriod 60*1000L  // период инверсии дисплея
#define INIT_KEY 50                // ключ первого запуска EEPROM. 0-254, на выбор
#define INIT_ADDR 0                // номер ячейки для хранения ключа
#define WDT_TIMEOUT 30             // 30 секунд отсутствия отклика для перезагрузки через WDT

//***********************************************************************************************************************
// Библиотеки
//***********************************************************************************************************************
#include <esp_task_wdt.h>       // библиотека WatchDogTimer
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <ArduinoOTA.h>         //бибилотека ОТА обновления по WiFi 
#include <GyverOLED.h>          //библиотека дисплея 
#include <Arduino.h>
#include <TimerMs.h>            // библиотека таймера
#include <GyverHub.h>           // GyverHub 
#include <EEPROM.h>             // стандартная библиотека управления энергонезависимой памятью

//******************************************************************************************************************
// Объекты библиотек
//******************************************************************************************************************
GyverOLED<SSH1106_128x64> oled;           // создание объекта экрана SSH1106 1,3''
HTTPClient http;                          // создаем объект http библиотеки HTTPClient
WiFiClient client;                        // создаем объект client библиотеки WiFiClient
TimerMs oledTmr(oledInvertPeriod, 1, 0);  // создаем объект oledTmr таймера TimerMs с периодом oledInvertPeriod
TimerMs heat4xTmr(heat4xPeriod, 1, 0);    // создаем объект heat4xTmr таймера TimerMs с периодом heat4xPeriod
TimerMs checkWifiTmr(checkWifiPeriod, 1, 0);   // создаем объект checkWifiTmr таймера TimerMs с периодом checkWifiPeriod
TimerMs sensorReadTmr(sensorReadPeriod, 1, 0); // создаем объект sensorReadTmr таймера TimerMs с периодом sensorReadPeriod
GyverHub hub;                                  // создаем объект GyverHub

//*********************************************************************************************************************
// Переменные
//**********************************************************************************************************************
float temperature;          // значение температуры
float humidity = 50;        // значение влажности
float tempTemperature;      // первичное значение температуры с датчика до проверки на выброс
float tempHumidity;         // первичное значение влажности с датчика до проверки на выброс
int8_t rssi;                // переменная измеренного значения rssi, dB
int8_t humCorrection = 0;   // поправка измеренного значения влажности
uint32_t heatTmr = 0;       // переменная таймера нагрева датчика SHT31
uint32_t openMonTmr = 0;    // переменная таймера отправки сообщений на сервер open-monitoring.online
uint32_t narodMonTmr = 0;   // переменная таймера отсылки данных на сервер NarodMon
bool heatFlag = 0;          // флаг нагрева датчика
bool oledFlag = 0;          // флаг состояния инверсии дисплея

//*********************************************************************************************************
// билдер GyverHub
//*********************************************************************************************************
void build(gh::Builder& b) {     
  b.Title(F("Климат на полке")); // добавим заголовок
  // добавляем горизонтальный контейнер
  // функции beginRow() и beginCol() всегда возвращают true
  if (b.beginRow()) {  
   b.Label_("Temp", temperature).label("Температура").color(gh::Colors::Red);
   b.Label_("Hum", humidity).label("Влажность").color(gh::Colors::Aqua);
   b.Label_("Rssi", rssi).label("RSSI").color(gh::Colors::Aqua);
   b.endRow();  
  }
  if (b.Slider(&humCorrection).range(-10, 10, 1).label("Поправка влажности").click()) { // слайдер
   EEPROM.write(1, humCorrection);    // записали измененное значение humCorrection
   EEPROM.commit();                   // сохранили изменения в эмулированной ЕЕПРОМ во флеш памяти для esp8266/esp32
  }; 
}  // end void build()

//**********************************************************************************************************************
// SETUP
//**********************************************************************************************************************
void setup() {

  esp_task_wdt_init(WDT_TIMEOUT, true);       //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                     //add current thread to WDT watch

  EEPROM.begin(2);                            // инициализация на ESP32 ЕЕРROM размером 2 байта  
  if (EEPROM.read(INIT_ADDR) != INIT_KEY) {   // в случае первого запуска
    EEPROM.write(INIT_ADDR, INIT_KEY);        // записали ключ  
    EEPROM.write(1, humCorrection);           // записали стандартное значение humCorrection
    EEPROM.commit();                          // сохранили изменения в эмулированной ЕЕПРОМ во флеш памяти для esp8266/esp32
   } else {
    humCorrection = EEPROM.read(1);           // восстановилии ранее записанное значение humCorrection
  }
  
  Serial.begin(115200);
  Wire.begin();                             // SensirionI2cSht3x.h and SensirionI2cSht4x.h 

  #ifdef USE_SHT31                         // если используется датчик SHT31
    sht3x.begin(Wire, SHT31_I2C_ADDR_44);  // SensirionI2cSht3x.h 
  #endif
  
  #ifdef USE_SHT41                         // если используется датчик SHT41
    sht4x.begin(Wire, SHT41_I2C_ADDR_44);  // SensirionI2cSht4x.h 
  #endif 

  oled.init();                  // инициализация дисплея   
  oled.setContrast(10);         // яркость 0..255
  oled.textMode(BUF_REPLACE);   // вывод текста на экран с заменой символов
  oled.invertDisplay(oledFlag); // вывод текста на экран с заменой символов
  
  initWiFi();                   // установили соединение WiFi

 // библиотека ArduinoOTA организует все нужное для ОТА прошивки
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.setHostname("ESP32_MeteoStation");
  ArduinoOTA.setPassword(otaPass);
  ArduinoOTA.begin();

  hub.mqtt.config("m6.wqtt.ru", 17108, mqttLogin, mqttPass);  // подключаем MQTT сервис
  // hub.mqtt.config("test.mosquitto.org", 1883);
  hub.config(hubPrefix, F("Basement"), F("f6d9"));
  hub.onBuild(build);
  hub.begin();
}  // end void Setup()

//***********************************************************************************************
// LOOP
//***********************************************************************************************
void loop() {

  esp_task_wdt_reset();  // сбрасываем Watch Dog Timer чтобы не прошла перезагрузка  
  
  ArduinoOTA.handle();  // Включаем поддержку ОТА

  hub.tick();                  // обязательно тикаем тут для нормальной работы конструктора интерфейса
  static gh::Timer tmr(2000);  // период 2 секунды  
  if (tmr) {                   // если прошел период
    hub.sendUpdate("Temp");    // обновляем значение температуры
    hub.sendUpdate("Hum");     // обновляем значение влажности
    hub.sendUpdate("Rssi");    // обновляем значение RSSI
  }
  
  #ifdef USE_SHT31                         // если используется датчик SHT31     
  // с периодом heatPeriod включаем прогрев датчика SHT31 на время heat3xTime
  // начальные значения heatFlag = 0, heatTmr = 0
  if (millis() - heatTmr >= (heatFlag ? heat3xTime : heatPeriod)) {       
   heatTmr = millis();                     // сброс таймера на начало нагрева датчика
   heatFlag = !heatFlag;                   // переключаем флаг состояния нагрева датчика
   if (heatFlag) sht3x.enableHeater();     
   else sht3x.disableHeater();       
  } // end If
  #endif  

  #ifdef USE_SHT41                         // если используется датчик SHT41
  // подогреваем датчик SHT41 если Humidity > heat4xBorder с периодом heat4xPeriod на 1 секунду  
  // или если пришло время ежесуточного прогрева датчика SHT41
  if (((humidity > heat4xBorder) && heat4xTmr.tick()) || ((millis() - heatTmr) > heatPeriod)) { 
    heatTmr = millis();                                              // сброс таймера на начало нагрева датчика
    sht4x.activateHighestHeaterPowerLong(temperature, tempHumidity); // SensirionI2cSht4x.h 
    humidity = tempHumidity + humCorrection;                         // SensirionI2cSht4x.h
    showScreen();                                                    // вывод показаний датчиков на экран
    delay(1000);                                                     // чтобы успеть увидеть цифры после нагрева    
  } // end If 
  #endif  

  if (oledTmr.tick()) {                               // если пришло время инвертировать дисплей
    oledFlag = !oledFlag;                             // инвертируем флаг состояния дисплея
    oled.invertDisplay(oledFlag);                     // инвертируем дисплей
  }

  // если пришло время опроса датчиков 
  if (sensorReadTmr.tick()){ 
    #ifdef USE_SHT31                         // если используется датчик SHT31
      sht3x.measureSingleShot(REPEATABILITY_HIGH, false, tempTemperature, tempHumidity); // SensirionI2cSht3x.h 
    #endif  
    #ifdef USE_SHT41                         // если используется датчик SHT41
      sht4x.measureHighPrecision(tempTemperature, tempHumidity);    // SensirionI2cSht4x.h    
    #endif  
    rssi = WiFi.RSSI();
    // если считанные показания разумны
    if ((tempTemperature < 100) && (tempTemperature > 5) && (tempHumidity < 93) && (tempHumidity > 20)) {
      temperature = tempTemperature;
      humidity = tempHumidity + humCorrection;
      showScreen();                      // вывод показаний датчиков на экран
    }    
  }  // end if 

  // восстанавливаем соединение при случайной пропаже  
  if (checkWifiTmr.tick() && (WiFi.status() != WL_CONNECTED)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    initWiFi();                          // установили соединение WiFi
  }
 
  // Если пришло время очередной отправки на open-monitoring.online и прошло заданное время с момента последнего нагрева датчика 
  if ((millis() - openMonTmr) >= openMonPeriod && (millis() - heatTmr) > (heat4xPeriod - 3000)) {
    openMonTmr = millis();               // сбрасываем таймер отправки данных  
    sendToOpenMon();                     // отправляем данные на open-monitoring.online
  }                                      // end if (sendtoOM)

  // Если пришло время очередной отправки на NarodMon и прошло заданное время с момента последнего нагрева датчика 
  if (((millis() - narodMonTmr) >= narodMonPeriod) && ((millis() - heatTmr) >= (heat4xPeriod - 3000))) {      
    narodMonTmr = millis();               // сбрасываем таймер отправки данных
    sendToNarodMon();                     // отправляем данные на NarodMon
  }                                       // end if (sendtonm)

}  // end Loop
