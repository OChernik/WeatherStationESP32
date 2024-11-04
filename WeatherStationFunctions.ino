// Функция устанавливает WiFi соединения
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
}  // end void initWiFi()

// функция showScreen() выводит на экран значения температуры, влажности, RSSI
// и оставшееся время до импульса нагрева датчика 
void showScreen() {
    oled.clear();                         // очищаем дисплей
    oled.setScale(2);                     // масштаб текста (1..4)
    oled.setCursor(0, 0);                 // курсор на начало 1 строки
    oled.print("H ");                     // вывод H 
    oled.print(humidity, 1);              // вывод значения Humidity
    oled.setCursor(0, 2);                 // курсор на начало 2 строки
    oled.print("T ");                     // вывод Т
    oled.print(temperature, 1);           // вывод значения Temperature
    oled.print(" ");                      // вывод " "
     
   #ifdef USE_SHT41                       // если используется датчик SHT41
    // counterDown это время, оставшееся до включения нагрева датчика SHT4x
    float counterDown = (heat4xPeriod - (millis() - heatTmr))/1000;  
    if (humidity > heat4xBorder) {       // если значение влажности больше heat4xBorder       
      oled.print(counterDown, 0);        // вывод значения времени до начала нагрева counterDown     
    }  // end IF  
   #endif
  
   #ifdef USE_SHT31                      // если используется датчик SHT31
    oled.print((heatFlag) ? " On" : " Off"); // вывод "On" если датчик греется
   #endif
  
    oled.setCursor(0, 4);                 // курсор на начало 3 строки
    oled.print("RSSI ");                  // вывод RSSI
    oled.print(rssi);                     // вывод значения RSSI.
    oled.update();                        // Вывод содержимого буфера на дисплей. Только при работе с буфером.
} // end showScreen

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
  String buf;                                                  // Буфер для отправки
  buf.reserve(90);                                             // резервируем память с небольшим запасом
  buf += F("http://open-monitoring.online/get?cid=2661&key="); // формируем заголовок
  buf += OpenMonKey;                                           // добавляем пароль пользователя 
  buf += F("&p1=");  
  buf += temperature;                                          // добавляем температуру 
  buf += F("&p2=");
  buf += humidity;                                             // добавляем влажность
  buf += F("&p4=");
  buf += rssi;                                                 // вывод силы сигнала Wi-Fi, dBm
  http.begin(buf.c_str());                                     // отправляем сформированную строку
  http.GET();                                                  // Send HTTP GET request
  http.end();                                                  // Free resources
}