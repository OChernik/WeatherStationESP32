// Датчик SHT41, SHT31  SDA - 21, SCL - 22
// Экран SSH1106 1,3''  SDA - 21, SCL - 22
//  
// Константы и дефайны________________________________________________________________________________________________________________
const char* ssid = "*****";       // WiFi login 
const char* password = "*****";   // WiFi password
const char* omKey = "*****";      // Open Monitoring Key
const char* hubPrefix = "*****";  // Gyverhub prefix

#define sensorReadPeriod 1000      // период между опросом датчика в мс.
#define openMonPeriod 5*60*1000L   // период между отправкой данных на сервер ОМ в мс.
#define narodMonPeriod 10*60*1000L // период между отправкой данных на сервер NM в мс.
#define checkWifiPeriod 30*1000L   // период проверки состояния WiFi соединения в мс.
#define heat3xPeriod 24*60*60*1000L// период включения нагрева датчика SHT3x (время МЕЖДУ включениями)
#define oledInvertPeriod 60*1000L  // период инверсии дисплея
#define heat3xTime 5*60*1000L      // время, на которое включается нагрев датчика SHT3x
#define heat4xPeriod 120*1000L     // период включения нагрева SHT4x 
#define heat4xBorder 75            // значение влажности, выше которого включается нагрев датчика SHT4x 
#define INIT_KEY 50                // ключ первого запуска EEPROM. 0-254, на выбор
#define INIT_ADDR 0                // номер ячейки для хранения ключа
#define WDT_TIMEOUT 30             // 30 секунд отсутствия отклика для перезагрузки через WDT

// Библиотеки______________________________________________________________________________________________________________
#include <esp_task_wdt.h>          // библиотека WatchDogTimer
#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
// #include <SensirionI2cSht3x.h>  // библиотека датчиков температуры и влажности SHT3х
#include <SensirionI2cSht4x.h>     // библиотека датчиков температуры и влажности SHT4х
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>            //бибилотека ОТА обновления по WiFi 
#include <GyverOLED.h>             //библиотека дисплея 
#include <Arduino.h>
#include <MyTimer.h>               // тестовая библиотека таймера
#include <GyverHub.h>              // GyverHub 
#include <EEPROM.h>                // стандартная библиотека управления энергонезависимой памятью

// Объекты библиотек_______________________________________________________________________________________________________
// SensirionI2cSht3x sht3x;               // создание объекта датчика sht3x библиотеки SensirionI2cSht3x
SensirionI2cSht4x sht4x;                  // создание объекта датчика sht4x библиотеки SensirionI2cSht4x
GyverOLED<SSH1106_128x64> oled;           // создание объекта экрана SSH1106 1,3''
HTTPClient http;                          // создаем объект http библиотеки HTTPClient
WiFiClient client;                        // создаем объект client библиотеки WiFiClient
MyTimer oledTmr(oledInvertPeriod);        // создаем объект oledTmr таймера MyTimer с периодом oledInvertPeriod
MyTimer heat4xTmr(heat4xPeriod);          // создаем объект heat4xTmr таймера MyTimer с периодом heat4xPeriod
MyTimer checkWifiTmr(checkWifiPeriod);    // создаем объект checkWifiTmr таймера MyTimer с периодом checkWifiPeriod
MyTimer sensorReadTmr(sensorReadPeriod);  // создаем объект sensorReadTmr таймера MyTimer с периодом sensorReadPeriod
GyverHub hub;                             // создаем объект GyverHub

// Переменные______________________________________________________________________________________________________________________
float temperature;          // значение температуры
float humidity = 50;        // значение влажности
float tempTemperature;      // первичное значение температуры с датчика до проверки на выброс
float tempHumidity;         // первичное значение влажности с датчика до проверки на выброс
int8_t rssi;                // переменная измеренного значения rssi, dB
int8_t humCorrection = 0;   // поправка измеренного значения влажности
uint32_t heat3xTmr = millis(); // переменная таймера нагрева датчика SHT31
uint32_t openMonTmr = 0;       // переменная таймера отправки сообщений на сервер open-monitoring.online
uint32_t narodMonTmr = 0;   // переменная таймера отсылки данных на сервер NarodMon
uint32_t heat4xStart = 0;    // переменная времени начала нагрева датчика SHT41
bool heatFlag = 0;          // флаг нагрева датчика
bool oledFlag = 0;          // флаг состояния инверсии дисплея

// билдер GyverHub___________________________________________________________________________________________________________________
void build(gh::Builder& b) {     
  b.Title(F("Климат на полке")); // добавим заголовок
  // добавляем горизонтальный контейнер  
  if (b.beginRow()) {            // функция beginRow()  всегда возвращает true
   b.Label_(F("Temp"), temperature).label(F("Температура")).color(gh::Colors::Red); // выводим метку со значением температуры
   b.Label_(F("Hum"), humidity).label(F("Влажность")).color(gh::Colors::Aqua);      // выводим метку со значением влажности
   b.endRow();  
  }
  if (b.Slider(&humCorrection).range(-10, 10, 1).label(F("Поправка влажности")).click()) { // слайдер
   EEPROM.write(1, humCorrection);    // записали измененное значение humCorrection
   EEPROM.commit();                   // сохранили изменения в эмулированной ЕЕПРОМ во флеш памяти для esp8266/esp32
  }; 
}  // end void build()

// SETUP________________________________________________________________________________________________________________________________
void setup() {

  esp_task_wdt_init(WDT_TIMEOUT, true);       //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);                     //add current thread to WDT watch

  EEPROM.begin(2);                            // инициализация на ESP32 ЕЕРROM размером 2 байта  
  if (EEPROM.read(INIT_ADDR) != INIT_KEY) {   // в случае первого запуска
    EEPROM.write(INIT_ADDR, INIT_KEY);        // записали ключ  
    EEPROM.write(1, humCorrection);           // записали стандартное значение humCorrection
    EEPROM.commit();                          // сохранили изменения в эмулированной ЕЕПРОМ во флеш памяти для esp8266/esp32
   } else {                                   // если ЕЕРROM уже ранее была инициализирована
    humCorrection = EEPROM.read(1);           // восстановили ранее записанное значение humCorrection
  }
  
  Serial.begin(115200);
  Wire.begin();                             // SensirionI2cSht3x.h and SensirionI2cSht4x.h 
  // sht3x.begin(Wire, SHT31_I2C_ADDR_44);  // SensirionI2cSht3x.h 
  sht4x.begin(Wire, SHT41_I2C_ADDR_44);     // SensirionI2cSht4x.h 
  
  oled.init();                  // инициализация дисплея   
  oled.setContrast(10);         // яркость 0..255
  oled.textMode(BUF_REPLACE);   // вывод текста на экран с заменой символов
  oled.invertDisplay(oledFlag); // вывод текста на экран с заменой символов
  
  initWiFi();                  // установили соединение WiFi

  // библиотека ArduinoOTA.h делает все нужное для работы ОТА
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

  ArduinoOTA.setHostname("ESP32_MeteoStation");  // устанавливаем название устройства, видимое в сети
  ArduinoOTA.begin();                            // запускаем ArduinoOTA

  hub.mqtt.config("test.mosquitto.org", 1883);     // конфигурируем MQTT сервер
  hub.config(hubPrefix, F("Basement"), F("f6d9")); // конфигурируем Gyverhub
  hub.onBuild(build);                              // запускаем Gyverhub
  hub.begin();                                     // запускаем Gyverhub
}  // end void Setup()

//LOOP______________________________________________________________________________________________________________________________
void loop() {

  esp_task_wdt_reset();  // сбрасываем Watch Dog Timer чтобы не прошла перезагрузка  
  
  ArduinoOTA.handle();   // включаем поддержку ОТА

  hub.tick();                  // тикаем для нормальной работы конструктора интерфейса
  static gh::Timer tmr(2000);  // период 2 секунды  
  if (tmr) {                   // если прошел период
    hub.sendUpdate("Temp");    // обновляем значение температуры
    hub.sendUpdate("Hum");     // обновляем значение влажности
  }
  
  // с периодом heatPeriod включаем прогрев датчика SHT31 на время heatTime
  // начальные значения heatFlag = 0, heatTmr = millis()
  // if (millis() - heat3xTmr >= (heatFlag ? heat3xTime : heat3xPeriod)) {       
  //  heat3xTmr = millis();                   // сброс таймера
  //  heatFlag = !heatFlag;                   // переключаем флаг состояния нагрева датчика
  //  (heatFlag) ? sht3x.enableHeater() : sht3x.disableHeater(); // переключаем нагрев датчика SensirionI2cSht3x.h  
  // } // end If

  // подогреваем датчик SHT41 если Humidity > heat4xBorder 
  // с периодом heat4xPeriod включаем прогрев датчика SHT41 на 1 секунду  
  if ((humidity > heat4xBorder) && heat4xTmr.tick()) { 
    heat4xStart = millis();                                          // сохраняем время начала нагрева датчика
    sht4x.activateHighestHeaterPowerLong(temperature, tempHumidity); // SensirionI2cSht4x.h 
    humidity = tempHumidity + humCorrection;                         // SensirionI2cSht4x.h
    showScreen();                                                    // вывод показаний датчиков на экран
    delay(1000);                                                     // чтобы успеть увидеть цифры после нагрева    
  } // end If

  if (oledTmr.tick()) {                               // если пришло время инвертировать дисплей
    oledFlag = !oledFlag;                             // инвертируем флаг состояния дисплея
    oled.invertDisplay(oledFlag);                     // инвертируем дисплей
  }
  
  // если пришло время опроса датчиков 
  if (sensorReadTmr.tick()){     
    // sht3x.measureSingleShot(REPEATABILITY_HIGH, false, tempTemperature, tempHumidity); // SensirionI2cSht3x.h 
    sht4x.measureHighPrecision(tempTemperature, tempHumidity);    // SensirionI2cSht4x.h    
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
  if ((millis() - openMonTmr) >= openMonPeriod && (millis() - heat4xStart) > (heat4xPeriod - 3000)) {
    openMonTmr = millis();               // сбрасываем таймер отправки данных  
    sendToOpenMon();                     // отправляем данные на open-monitoring.online
  }                                      // end if (sendtoOM)

  // Если пришло время очередной отправки на NarodMon и прошло заданное время с момента последнего нагрева датчика 
  if (((millis() - narodMonTmr) >= narodMonPeriod) && ((millis() - heat4xStart) >= (heat4xPeriod - 3000))) {      
    narodMonTmr = millis();               // сбрасываем таймер отправки данных
    sendToNarodMon();                     // отправляем данные на NarodMon
  }                                       // end if (sendtonm)

}  // end Loop
//____________________________________________________________________________________________________________________________________

// Функции____________________________________________________________________________________________________________________________
// Функция для установки WiFi соединения 
void initWiFi() {                             
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
} // end void initWiFi()

// функция отправляет данные на сервер NarodMon
void sendToNarodMon() {
    String buf;                           // Буфер для отправки
    buf += F("#ESP32");
    buf += WiFi.macAddress();
    buf += F("\n");
    buf.replace(":", "");                 // убираем из строки символы ":"
    buf += F("#Temp1#");
    buf += temperature;
    buf += F("#Подвал\n");                //NarodMon: вывод температуры подвала
    buf += F("#RH1#");
    buf+= humidity;
    buf += F("#Подвал\n");                //NarodMon: вывод влажности подвала
    buf += F("#DBM#");
    buf += rssi;
    buf += F("#Подвал\n");                //NarodMon: вывод силы сигнала Wi-Fi, dBm
    buf += F("##\n");                     //NarodMon: закрываем пакет
    client.connect("narodmon.ru", 8283);  //NarodMon: Подключаемся
    client.print(buf.c_str());            // И отправляем данные в сеть
    client.stop();                        // Разрываем соединение с сервером
}

// функция отправляет данные на сервер open-monitoring.online
void sendToOpenMon() {
    String buf;                                                            // Буфер для отправки
    buf.reserve(90);                                                       // резервируем память с небольшим запасом
    buf += F("http://open-monitoring.online/get?cid=2661&key=);            // формируем заголовок
    buf += omKey;                                                          // добавляем пароль пользователя 
    buf += F("&p1=");  
    buf += temperature;                                                    // добавляем температуру 
    buf += F("&p2=");
    buf += humidity;                                                       // добавляем влажность
    buf += F("&p4=");
    buf += rssi;                                                        // вывод силы сигнала Wi-Fi, dBm
    http.begin(buf.c_str());                                            // отправляем сформированную строку
    http.GET();                                                         // Send HTTP GET request
    http.end();                                                         // Free resources
}

// функция showScreen() выводит на экран значения температуры, влажности, пинг WiFi, RSSI
// и оставшееся время до импульса нагрева датчика 
void showScreen() {
    // counterDown это время, оставшееся до включения нагрева датчика SHT4x
    float counterDown = (heat4xPeriod - (millis() - heat4xTime))/1000;  
    oled.clear();                         // очищаем дисплей
    oled.setScale(2);                     // масштаб текста (1..4)
    oled.setCursor(0, 0);                 // курсор на начало 1 строки
    oled.print("H ");                     // вывод H 
    oled.print(humidity, 1);              // вывод значения Humidity
    oled.setCursor(0, 2);                 // курсор на начало 2 строки
    oled.print("T ");                     // вывод Т
    oled.print(temperature, 1);           // вывод значения Temperature

    if (humidity <= heat4xBorder) {       // если значение влажности меньше heat4xBorder
      oled.print((heatFlag) ? " On" : " Off"); // вывод "On" если датчик греется
     } else {                             // если значение влажности больше heat4xBorder
      oled.print(" ");                    // вывод " " если датчик будет греться
      oled.print(counterDown, 0);         // вывод значения времени до начала нагрева counterDown     
    }  // end IF

    oled.setCursor(0, 4);                 // курсор на начало 3 строки

    if (Png > 0) {                        // если существует значение пинга
      oled.print("Ping ");                // вывод Ping           
      oled.print(Png);                    // вывод времени пинга       
     } else {
      oled.print(" Ping Error !");        // вывод сообщения " Ping Error !"
    }  // end IF

    oled.setCursor(0, 6);                 // курсор на начало 4 строки
    oled.print("RSSI ");                  // вывод RSSI
    oled.print(rssi);                     // вывод значения RSSI.
    oled.update();    // Вывод содержимого буфера на дисплей. Только при работе с буфером.
} // end showScreen
