// Датчик HDC1080       SDA - 21, SCL - 22
// Датчик SHT31D        SDA - 21, SCL - 22
// Экран SSH1106 1,3''  SDA - 21, SCL - 22
// Esp32 с антенной 192.168.31.107, ESP324022D803F9F4
//
#define sensorReadPeriod 2000 // период между опросом датчика в мс.
#define openMonPeriod 300000  // период между отправкой данных на сервер ОМ в мс.
#define narodMonPeriod 600000 // период между отправкой данных на сервер NM в мс.
#define checkWifiPeriod 30000 // период проверки состояния WiFi соединения в мс.
#define PingPeriod 63000      // период измерения пинга
#define INIT_KEY 50           // ключ первого запуска EEPROM. 0-254, на выбор
#define INIT_ADDR 0           // номер ячейки для хранения ключа
#define WDT_TIMEOUT 30        // 30 секунд отсутствия отклика для перезагрузки через WDT

#include <esp_task_wdt.h>     // библиотека WatchDogTimer
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
// #include "ClosedCube_HDC1080.h"
#include "Adafruit_SHT31.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <GyverOLED.h> //библиотека дисплея 
#include <Arduino.h>
// #include <GyverHub.h>
#include <EEPROM.h>
#include <ESP32Ping.h>    // библиотека проверки пинга
// #include <EEManager.h>  // Gyver lib 

// ClosedCube_HDC1080 hdc1080;             // создание объекта датчика HDC1080
Adafruit_SHT31 sht31 = Adafruit_SHT31();   // создание объекта датчика sht31d
GyverOLED<SSH1106_128x64> oled;            // создание объекта экрана SSH1106 1,3''
// GyverHub hub("ChernikDevices", "Basement_ESP32", "");

float Temperature;
float Humidity;
uint8_t Png = 1;              // переменная измеренного значения пинга, мс
int8_t rssi;
int8_t humCorrection = 1 ;  // начальная поправка измеренного значения влажности
uint32_t sensorReadTmr = 0; // переменная таймера опроса датчиков
uint32_t openMonTmr = 0;    // переменная таймера отсылки данных на сервер Open Monitoring
uint32_t narodMonTmr = 0;   // переменная таймера отсылки данных на сервер NarodMon
uint32_t checkWifiTmr = 0;  // переменная таймера  соединения WiFi
uint32_t PingTmr = 0;       // переменная таймера пинга


// EEManager memory(humCorrection, 2000); // передаём переменную в менеджер EEPROM. 2000 ms таймаут обновления

// const char* ssid = "zelenaya.net3";
// const char* password = "18fev1985";
const char* ssid = "Koltsevaya_15";
const char* password = "vibrometr";
// WiFiServer server(80);

// это наш билдер. Он будет вызываться библиотекой
// для сборки интерфейса, чтения значений и проч.
// void build() {
//     // добавим заголовок
//     hub.Title(F("Климат на полке"));
//     // BeginWidgets() начинает новую горизонтальную строку виджетов
//     hub.BeginWidgets();
//      // сменим ширину на 100%
//     hub.WidgetSize(100);
//     // сделаем ещё один label с личным именем, к функции добавится _
//     // ниже в loop будем отправлять обновления на его имя
//     hub.Label_(F("Temp"), String(Temperature), F("Температура"));     // метка со значением температуры
//     hub.Label_(F("Hum"), String(Humidity), F("Влажность"));           // метка со значением влажности
//     hub.Label_(F("Rssi"), String(rssi), F("Сигнал WiFi"));            // метка со значением уровня сигнала
//     hub.Label_(F("Ping"), String(Png), F("Пинг до роутера, мс"));     // метка со значением времени пинга
//     hub.Label_(F("DeltaHum"), String(humCorrection), F("Поправка датчика влажности")); // метка с поправкой влажности
//     // Добавили слайдер с подключенной переменной humCorrection
//     if (hub.Slider(&humCorrection, GH_INT8, F("Поправка датчика влажности, %"), -10, 10, 1)) {
//      // Добавили крутилку с подключенной переменной humCorrection
//      // if (hub.Spinner(&humCorrection, GH_INT8, F("Поправка датчика влажности, %"), -10, 10, 1)) {
//      // Добавили текстовый ввод с подключенной переменной humCorrection
//      // if (hub.Input(&humCorrection, GH_INT8, F("Поправка датчика влажности, %"))) {  
//       hub.sendUpdate("DeltaHum", String(humCorrection)); // обновляем значение метки с humCorrection
//       EEPROM.write(1, humCorrection);          // записали измененное значение humCorrection
//       EEPROM.commit();                         // сохранили изменения в эмулированной ЕЕПРОМ во флеш памяти для esp8266/esp32      
//     }    
//     hub.EndWidgets();    
// }   // end void build()

void initWiFi() {                             // Функция для установки WiFi соединения 
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
} // end void initWiFi()

void setup() {

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);               //add current thread to WDT watch

  EEPROM.begin(2);                          // инициализация на ESP32 ЕЕРROM размером 2 байта
  // запускаем менеджер EEPROM, указав адрес и ключ запуска. Он сам проверит ключ, 
  // а также прочитает данные из EEPROM (если они там есть) и запишет в переменную.
  // memory.begin(0, INIT_KEY);   // передаем стартовый адрес записи и ключ 
  
  if (EEPROM.read(INIT_ADDR) != INIT_KEY) {   // в случае первого запуска
    EEPROM.write(INIT_ADDR, INIT_KEY);        // записали ключ  
    EEPROM.write(1, humCorrection);           // записали стандартное значение humCorrection
    EEPROM.commit();                          // сохранили изменения в эмулированной ЕЕПРОМ во флеш памяти для esp8266/esp32
  }  
  
  Serial.begin(115200);
  // hdc1080.begin(0x40);
  sht31.begin(0x44);           // I2C address: 0x44 or 0x45
  oled.textMode(BUF_REPLACE);  // вывод текста на экран с заменой символов
  oled.init();                 // инициализация дисплея
  
  initWiFi();                  // установили соединение WiFi

  // server.begin();
  
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
  ArduinoOTA.begin();

  // hub.setupMQTT("test.mosquitto.org", 1883);
  // hub.onBuild(build);     // подключаем билдер
  // hub.begin();            // запускаем систему
}  // end void Setup()

void loop() {

  esp_task_wdt_reset();  // сбрасываем Watch Dog Timer чтобы не прошла перезагрузка  
  
  ArduinoOTA.handle();  // Включаем поддержку ОТА

  // hub.tick();           // обязательно тикаем тут для нормальной работы конструктора интерфейса

  // memory.tick();        // здесь произойдёт запись EEPROM по встроенному таймеру
   
  if (millis() - PingTmr >= PingPeriod) {   // периодически измеряем время пинга  
    PingTmr = millis();                     // сброс таймера
    bool p = Ping.ping(WiFi.gatewayIP());   // пингуем роутер  
    if (p) {                                // если пинг проходит  
      Png = (uint8_t)Ping.averageTime();    // присваиваем среднее время пинга
    } else {
      WiFi.disconnect();
      initWiFi();                           // установили соединение WiFi      
    }    
  } // end If
  
  // static GHtimer tmr(1000);    // обновим метки с именами Temp, Hum, Rssi, Ping, DeltaHum по таймеру каждую 1 секунду
  
  // if (tmr) hub.sendUpdate("Temp");
  // if (tmr) hub.sendUpdate("Hum");
  // if (tmr) hub.sendUpdate("Rssi");
  // if (tmr) hub.sendUpdate("Ping");
  // if (tmr) hub.sendUpdate("DeltaHum");
  
  if (millis() - sensorReadTmr >= sensorReadPeriod){     // если пришло время опроса датчиков
    sensorReadTmr = millis();                            // сброс таймера
    float tempTemperature = sht31.readTemperature();
    float tempHumidity = sht31.readHumidity() + humCorrection;
    // float tempTemperature = hdc1080.readTemperature();
    // float tempHumidity = hdc1080.readHumidity() + humCorrection;
    rssi = WiFi.RSSI();
    // если считанные показания разумны
    if ((tempTemperature < 30) && (tempTemperature > 5) && (tempHumidity < 93) && (tempHumidity > 30)) {
      Temperature = tempTemperature;
      Humidity = tempHumidity;
      showScreen();                      // вывод показаний датчиков на экран
    }    
  }  // end if 

  // восстанавливаем соединение при случайной пропаже  
  if ((millis() - checkWifiTmr >= checkWifiPeriod) && (WiFi.status() != WL_CONNECTED)) {
    checkWifiTmr = millis();
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    initWiFi();                          // установили соединение WiFi
  }
 
  if (millis() - openMonTmr >= openMonPeriod) {                            // Если прошло время, и можно отправлять - работаем.
    openMonTmr = millis();                                                 // сбрасываем таймер отправки данных
    String buf;                                                            // Буфер для отправки
    buf += F("http://open-monitoring.online/get?cid=2661&key=K9nZKZ&p1="); //OpenMonitoring: формируем заголовок
    buf += Temperature;                                                    //OpenMonitoring: вывод температуры подвала
    buf += F("&p2=");
    buf += Humidity;                                                       //OpenMonitoring: вывод влажности подвала
    buf += F("&p4=");
    buf += rssi;                                                        //OpenMonitoring: вывод силы сигнала Wi-Fi, dBm
    HTTPClient http;                                                    // создаем объект HTTPClient
    http.begin(buf.c_str());                                            // отправляем сформированную строку
    int httpResponseCode = http.GET();                                  // Send HTTP GET request
    http.end();                                                         // Free resources
  }                                                                     // end if (sendtoOM)

  if (millis() - narodMonTmr >= narodMonPeriod) {      // Если прошел заданный период и можно отправлять - работаем.
    narodMonTmr = millis();                            // сбрасываем таймер отправки данных
    String buf;                                        // Буфер для отправки
    buf += F("#ESP32");
    buf += WiFi.macAddress();
    buf += F("\r\n");
    buf.replace(":", "");                               // ESP324022D803F9F4  // идентификатор прибора
    buf += F("#Temp1#");
    buf += Temperature;
    buf += F("#Подвал\r\n");                            //NarodMon: вывод температуры подвала
    buf += F("#RH1#");
    buf+= Humidity;
    buf += F("#Подвал\r\n");                            //NarodMon: вывод влажности подвала
    buf += F("#DBM#");
    buf += rssi;
    buf += F("#Подвал\r\n");                            //NarodMon: вывод силы сигнала Wi-Fi, dBm
    buf += F("##\r\n");                                 //NarodMon: закрываем пакет
    WiFiClient client;
    client.connect("narodmon.ru", 8283);  //NarodMon: Подключаемся
    client.print(buf);                    // И отправляем данные в сеть
  }                                       // end if (sendtonm)

}  // end Loop

// Процедура showScreen() выводит на экран значения температуры, влажности, пинг WiFi и значение RSSI
void showScreen() {
    oled.clear();                         // очищаем дисплей
    oled.setScale(2);                     // масштаб текста (1..4)
    oled.setCursor(0, 0);                 // курсор на начало 1 строки
    oled.print("Hum  ");                  // вывод Hum 
    oled.print(Humidity, 1);              // вывод значения Humidity
    oled.setCursor(0, 2);                 // курсор на начало 2 строки
    oled.print("Temp ");                  // вывод Темп
    oled.print(Temperature, 1);           // вывод значения Temperature.
    oled.setCursor(0, 4);                 // курсор на начало 3 строки
    if (Png > 0) {                        // если существует значение пинга
       oled.print("Ping ");               // вывод Ping           
       oled.print(Png);                   // вывод времени пинга       
     } else {
       oled.print(" Ping Error !");       // вывод сообщения " Ping Error !"
    }  // end IF
    oled.setCursor(0, 6);                 // курсор на начало 4 строки
    oled.print("RSSI ");                  // вывод RSSI
    oled.print(rssi);                     // вывод значения RSSI.
    oled.update();    // Вывод содержимого буфера на дисплей. Только при работе с буфером.
} // end showScreen