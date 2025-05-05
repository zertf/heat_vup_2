//Программа регулятора нагрева

// ??? используется если я не понял почему так, при этом не требуется дополнительная инфа из раздела обучение в Todoist
// НИ - не используется (убрать в backup как будет возможность), может где-то не стоять

//Библиотеки
#include <Arduino.h> //Библиотека стандартных команд Arduino
#include "PWMrelay.h" //Библиотека работы твердотельного реле
#include "GyverRelay.h" //Библиотека релейного регулирования. НИ
#include <QuickPID.h> //Библиотека ПИД регулирования
//#include <EasyButton.h> //Библиотека приема сигналов с кнопок НИ
#include <LiquidCrystal_I2C.h> //Библиотека для i2c дисплеев НИ
#include <Preferences.h> //Библиотека для работы с энергонезависимой памятью конкретно в ESP32
#include "MAX31855.h" //Библиотека Роба Тилларта MAX31855_RT
// #include "MAX6675.h" //Библиотека Роба Тилларта MAX6675 НИ
#include <WiFi.h> //Библиотека для работы с WiFi
#include <HTTPClient.h> //Библиотека для отправки post запроса на ntfy НИ
#include <AsyncTCP.h>           //Какая-то служебная библиотека для WebSerialLite.h
#include <ESPAsyncWebServer.h>  //Какая-то служебная библиотека для WebSerialLite.h
#include <WebSerialLite.h>      //Библиотека Serial по WEB (облегченная версия, то что нужно)
//#include <GyverMAX6675_SPI.h>   //Библиотека MAX6675 НИ
//#include "Adafruit_MAX31855.h"  //Библиотека MAX31855 от Adafruit НИ

//Библиотки для обменна данными с RPi
//Библиотека для работы с WiFi подключена выше
#include <WiFiUdp.h> //Библиотека для UDP протокола

//ТЮНЕР https://github.com/Dlloydev/sTune/blob/main/examples/MAX6675_PTC_SSR/Get_All_Tunings/Get_All_Tunings.ino
#include <sTune.h>


//Месточисления для статуса ??? зачем нужны + чтение про define
#define Off 0
#define On 1
#define Done 2 //Не используется
#define Err 3

//Объявление констант

// user settings
uint32_t settleTimeSec = 10;
uint32_t testTimeSec = 310;  // sample interval = testTimeSec / samples Конастанта времени
const uint16_t samples = 500;
const float inputSpan = 575;
const float outputSpan = 255;
float outputStart = 0;
float outputStep = 50;
float tempLimit = 650;

//Константы пинов

//Не используемые НИ
const byte HeatOn_s_pin = 33; //Пин для включенного положения переключателя (пока что не используется)
const byte HeatOff_s_pin = 25; //Пин для выключенного положения переключателя (пока что не используется)
const byte Done_pin = 26; //Светодиод готовности (с появлением экрана стал не нужен) (пока что не используется)
const byte Leak_pin1 = 13; //Пин первого датчика протечки
const byte Leak_pin2 = 14; //Пин второго датчика протечки
const byte Leak_pin3 = 27; //Пин третьего датчика протечки
const byte Relay_pin = 21; //Управление реле нагрева (21 под i2c что как раз запрещает, ибо убрали обычное реле)


/*
const byte Solid_relay_pin = 32; //Управление твердотельным реле. Сменить имя на SSR
const byte tc_pod_CS = 17; //Пин CS (SS) для подложкодержателя
const byte tc_nag_CS = 16; //Пин CS (SS) для нагревателя
const byte tc_ctrl_CS = 4; //Пин CS (SS) для контрольной термопары
const byte tc_mag_CS = 5; //Пин CS (SS) четвёртого датчика подожки (магнетрон)
const byte SPI_SO = 2; //Пин SPI
const byte SPI_SCK = 15; //Пин SPI
//21 и 22 пины заняты под i2c
*/
const byte Solid_relay_pin = 32; //Управление твердотельным реле. Сменить имя на SSR
const byte tc_pod_CS = 17; //Пин CS (SS) для подложкодержателя
const byte tc_nag_CS = 16; //Пин CS (SS) для нагревателя
const byte tc_ctrl_CS = 4; //Пин CS (SS) для контрольной термопары
const byte tc_mag_CS = 23; //Пин CS (SS) четвёртого датчика подожки (магнетрон) 5
const byte SPI_SO = 22; //Пин SPI 2
const byte SPI_SCK = 21; //Пин SPI 15
//21 и 22 пины заняты под i2c

//Константы для сглаживания значения термопар
const byte meas_count = 1; //Окно скользящего среднего для значений температуры (мб логично что это должно быть в пределах от 3 до 8, но если значения стабильны, то можно и 1). Чем больше, тем медленнее реакция системы, но лучше сглаживание.
const float meas_rate = (2.0 / (meas_count + 1.0)); //Используется для вычисления скользящего среднего, при meas_count = 1 ничего не делает

//Параметры сети для передачи данных
const char* ssid = "ИЗМЕНИТЬ на имя сети"; //SSID сети
const char* password = "ИЗМЕНИТЬ на пароль сети"; //WiFi Password

//Объявим переменные
byte Status = Off; //Переменная статуса нагрева; 0 - выкл, 1 - вкл, 2 - достигло цели, поддержание температуры
byte Status_leak = 0; //Переменная статуса протечек; 0 - протечки нет, 1 - датчик зафиксировал протечку, 2 - 2 датчика зафиксировало протечку, 3 - 3 датчика
//byte i = 0;//Переменная счётчика (для отладки)
unsigned long time_print = 2000; //Время таймера выводы на экран. Экрана нет
unsigned long time_serial = 3000; //Время таймера вывода информации по serial. ??? 3 секунды
unsigned long time_UDP = 2000; //Время таймера вывода информации по UDP
bool Serial_st = On; //Включает - выключает Serial
bool WebSerial_st = On; //Включает - выключает WebSerial
bool Display_st = Off; //Включает - выключает обновление дисплея
bool Notify_st = Off; //Включает - выключает отправку уведомлений об экстренных ситуациях
bool Leak_notify_sended = 0; //Переменная для контроля отправки уведомления о протечке
bool trigger = false; //Для проверки разницы значений команд в беспроводной передаче данных

//Переменные связанные с нагревом
float hyst_t = 5; //Гистерезис температуры
unsigned long goal_t = 100; //Целевая температура
float curr_t_pod = 10; //Текущая температура подложки
float curr_t_nag = 10; //Текущая температура нагревателя
float curr_t_ctrl = 10; //Текущая температура контрольной термопары
float curr_t_mag = 10; //Текущая температура термопары магнетрона

float curr_t_pod_d = 10; //Текущая температура датчика термопары подложки
float curr_t_nag_d = 10; //Текущая температура датчика термопары нагревателя
float curr_t_ctrl_d = 10; //Текущая температура датчика термопары нагревателя
float curr_t_mag_d = 10; //Текущая температура датчика термопары магнетрона

//Переменные для ПИД регулирования
float Setpoint, Output, Input;

//Константы для ПИД регулирования. Вычислены тюнером, но тербуют доработки (для 130 Вольт на БП)
const float Kp = 4.070; //4.070 was
const float Ki = 0.003; //0.003 was
const float Kd = 0.046; //0.046 was

unsigned long relay_status = 2; //Статус реле (0 - постоянно выкл, 1 - постоянно вкл, 2 - регулирование, 3 - регулирование без изменения коэффициенттов)

unsigned long period = 0; //Вспомогательная для расчётов связанных с реле. Мб сменить тип


unsigned long time_relay = 2000; //Время изменения состояния реле в мс. От него сильно зависит точность. Нужно подбирать оптимальное значение. НИ

unsigned long time_tc = 230; //Время таймера опроса датчиков термопары (не может быть меньше 200 мс). Должно быть меньше time_relay.

float k_t = 0.5; //Коэффициент обратной связи (нужно подобрать)
unsigned long tmr_relay = 0; //Таймер изменения состояния реле (на millis, ручной)


//Переменные связанные с передачей данных
char packetBuffer[16]; //Пакет для получения данных. Не путать с buf[256]
char previousBuffer[16]; //Пакет для получения данных. Не путать с buf[256]
unsigned int localPort = 9999;
const char *serverip = "192.168.0.100"; //IP адрес RPi. Нужно проверять и менять время от времени. Возможно стоит ввсести команду для ввода пока не реализован post-get.
//const char *serverip = "192.168.1.2"; 
//Возможно сделать переменной для энергозависимой памяти
unsigned int serverport = 8888;
unsigned int debugport = 8889;





//Задание классов
PWMrelay Solid_relay(Solid_relay_pin, true, time_tc); //Задание класса регулятора твердотельного реле. True это прямое, т.к. по умолчанию почему-то обратное
GyverRelay regulator(REVERSE); //Задание класса регулятора

QuickPID PIDregulator(&Input, &Output, &Setpoint);

LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4); //Задание класса экрана
Preferences pref; //Задание класса энергонезависимой памяти
AsyncWebServer server(80); //Задание класса вебсервера на 80 порту
//GyverMAX6675_SPI<tc_pod_CS> tc_pod; //Задание класса для датчика температуры подложки
//Adafruit_MAX31855 tc_nag(SPI_SCK, tc_nag_CS, SPI_SO); //Задание класса датчика термопары нагревателя
//Adafruit_MAX31855 tc_pod(SPI_SCK, tc_pod_CS, SPI_SO); //Задание класса датчика термопары подложкодержателя
MAX31855 tc_pod(tc_pod_CS, SPI_SO, SPI_SCK); //Задание класса датчика термопары подложкодержателя
MAX31855 tc_nag(tc_nag_CS, SPI_SO, SPI_SCK); //Задание класса датчика термопары нагревателя
//MAX6675 tc_ctrl(tc_ctrl_CS, SPI_SO, SPI_SCK); //Задание класса датчика контрольной термопары
MAX31855 tc_ctrl(tc_ctrl_CS, SPI_SO, SPI_SCK); //Задание класса датчика контрольной термопары
MAX31855 tc_mag(tc_mag_CS, SPI_SO, SPI_SCK); //Задание класса датчика термопары на магнетроне

//Класс (объект) для обмена с RPi
WiFiUDP udp;
WiFiUDP udp_data;

sTune tuner = sTune(&Input, &Output, tuner.ZN_PID, tuner.directIP, tuner.printALL);
sTune tuner5T = sTune(&Input, &Output, tuner.ZN_PID, tuner.direct5T, tuner.printALL);

//Создание класса Таймер с возможностью автоперезапуска и паузы
class Timer {
  /* Класс Таймера
  В слассе имеются следующие функции:
  1. Старт таймера (принимает на вход период);
  2. Старт таймера (без данных на входе просто перезагружает таймер);
  2. Стоп таймера (не реализовано так как не используется);
  3. Пауза;
  4. Продолжение после паузы;
  5. Продолжение после паузы с аргументом на случай перезапуска;
  6. Функция срабатывания (ничего не принимает на вход, возвращает кончился таймер или нет);
  7. Функция срабатывания (принимает на вход bool на случай если при срабатывании требуется сразу перезапустить таймер);
  8. Функция оставшегося времени (ничего не принимает, возвращает сколько времени осталось в мс);
  Класс должен быть полностью самодостаточным. Все операции со временем должны производиться через него.*/
  public:
  Timer () {}
  void start (unsigned long length_in) { //Процедура старта таймера с указанием времени
    _length_st = length_in;
    start();
  }
  void start () { //Процедура старта таймера с сохранившимся временем последнего старта
    _length = _length_st;
    _stop_t = millis() + _length;
    _is_pause = 0;
  }

  //void stop () {}

  void pause () { //Функция паузы
    if (!_is_pause && !ready()) {
      _is_pause = 1;
      _length = _stop_t - millis();
    }
  }
  void go () { //Функция продолжения после паузы
    if (_is_pause) {
      _is_pause = 0;
      _stop_t = millis() + _length;
    } else start();
  }
  void go (unsigned long length_in_go) { //Функция продолжения после паузы c аргументом на случай старта
    if (_is_pause) {
      _is_pause = 0;
      _stop_t = millis() + _length;
    } else start(length_in_go);
  }
  bool ready () { //Функция срабатывания без перезапуска
    if (!length()) {
      return 1;
    }
    return 0;
  }
  bool ready (bool restart) { //Функция срабатывания с перезапуском
    if (!length()) {
      if (restart) start();
      return 1;
    }
    return 0;
  }

  unsigned long length () { //Функция вычисления оставшегося времени (главнее функции срабатывания)
    if (!_is_pause) {if (millis() >= _stop_t) {return 0;} else {return (_stop_t - millis());}} else return _length;
  }

  void add (unsigned long length_add, bool restart_add) {
    pause();
    _length = _length + (length_add);
    if (restart_add) go();
  }

  private:
  //Приватные переменные
  unsigned long _length_st = (5L * 60L * 1000L); //Стандартная длина таймера
  unsigned long _length = _length_st; //Текущая длина таймера и оставшееся время
  unsigned long _stop_t = 0; //время предполагаемого окончания (расчитывается от длины и времени начала, которое не хранится)
  bool _is_pause = 0; //Переменная статуса паузы
};

//Создание таймеров
//Timer tmr_1; //Таймер на 1 секунду для вывода логов и отладки
Timer tmr_print; //Таймер для вывода инфы на экран
Timer tmr_serial_put; //Таймер для выводы инфы в сериал и webserial
Timer tmr_UDP_put; //Таймер для передачи информации через UDP
Timer tmr_tc_get; //Таймер для опроса датчиков термопары


//Описание функций
void Status_check();          //Функция проверки статуса (требует переработки)
void Leak_test();             //Функция проверки пинов протечки. Проверяет пины на наличие сигнала и в соответствии с этим изменяет статус протечки.
void regulator_check();       //Функция управления реле (на таймере внутри функции)
void print_display();         //Функция вывода информации на lcd экран (на таймере)
void serialget();   //Функция приема сигналов по Serial
void serialput();             //Функция отправки сигналов по Serial (на таймере)
void webserialput();          //Функция отправки сигналов по WebSerial (на таймере)
void webserialget(uint8_t *data, size_t len); //Функция приема сообщений по WebSerial
bool executer(char key, unsigned long val);   //Функция выполнения полученных команд. Исполняет полученные через serial команды. На вход принимает ключ и число.
void infoserialput();         //Функция вывода полной информации в Serial
void infowebserialput();      //Функция вывода полной информации в WebSerial
void tcget31855();            //Функция получения и сглаживания данных термопар (на таймере)
int leak_notification ();    //Функция отправки уведомлений при детектировании протечки
void temp_notification ();    //Функция отправки уведомлений при детектировании экстремальных температур

//Функция проверки статуса (требует переработки)
void Status_check() {
  if ((curr_t_pod > goal_t + 30) or (curr_t_pod_d > 120) or (curr_t_nag_d > 120) or (curr_t_pod > 700) or (curr_t_nag > 700) or (curr_t_pod < 5) or (curr_t_nag < 5)) {
    Status = Err;
    if (Notify_st) {temp_notification ();}
  } else if (curr_t_pod > goal_t - hyst_t) {
    Status = Done;
  }
}

//Функция проверки пинов протечки. Проверяет пины на наличие сигнала и в соответствии с этим изменяет статус протечки.
void Leak_test() {
  Status_leak = 0;
  if (touchRead(Leak_pin1) == 0) {Status_leak += 1;} //0 на touchRead означает полную замкнутость, тоесть протечку
  if (touchRead(Leak_pin2) == 0) {Status_leak += 1;}
  if (touchRead(Leak_pin3) == 0) {Status_leak += 1;}
  if (Status_leak and Notify_st and (not Leak_notify_sended)) {
    //leak_notification();
    if (leak_notification() == 200) {Leak_notify_sended = 1;} //Переменной присваивается 1, чтобы уведомления не сыпались кучей
  }
}

//Функция управления реле (на таймере внутри функции)
void regulator_check() {
  if (millis() - tmr_relay >= time_relay) {
    //возможно здесь стоит считывать с датчиков текущую температуру, а может и не здесь
    regulator.input = curr_t_pod;   // сообщаем регулятору текущую температуру
    digitalWrite(Relay_pin, regulator.compute((millis() - tmr_relay)/1000));  //отправляем сигнал на реле
    tmr_relay = millis();
  }
}


//Функция вывода информации на lcd экран (на таймере)
void print_display() {
  lcd.setCursor(7, 0);
  lcd.print(curr_t_pod, 3);
  lcd.setCursor(7, 1);
  lcd.print(curr_t_nag, 3);

}

//Функция приема сигналов по Serial
void serialget() {
  if (Serial.available() > 0) {
    char key = Serial.read();
    unsigned long val = Serial.parseInt();
    executer(key, val);
  }
  else if (trigger) {//Доп условие на всякий
    memset(previousBuffer, 0, sizeof(previousBuffer)); //Обнуление на всякий
    memcpy(previousBuffer, packetBuffer, sizeof(packetBuffer));  // ← правильная замена
    char key = tolower(packetBuffer[0]);
    unsigned long val = atoi(packetBuffer+1);
    executer(key, val);
    trigger = false;    
  }
}

//Функция отправки сигналов по Serial (на таймере)
void serialput() {
  Serial.print(curr_t_pod, 0);
  Serial.print("; ");
  Serial.print(curr_t_nag, 0);
  Serial.print("; ");
  Serial.print(curr_t_ctrl, 0);
  Serial.print("; ");
  Serial.print(curr_t_mag, 0);
  Serial.print("; ");
  Serial.print(curr_t_pod_d, 0);
  Serial.print("; ");
  Serial.print(curr_t_nag_d, 0);
  Serial.print("; ");
  Serial.print(curr_t_ctrl_d, 0);
  Serial.print("; ");
  Serial.print(curr_t_mag_d, 0);
  Serial.print("; ");
  Serial.print(goal_t);
  Serial.print("; ");
  Serial.print(relay_status);
  Serial.print("; ");
  Serial.print(Status_leak);
  Serial.println("; ");
}

//Функция отправки сигналов по WebSerial (на таймере)
void webserialput() {
  WebSerial.print(curr_t_pod);
  WebSerial.print("; ");
  WebSerial.print(curr_t_nag);
  WebSerial.print("; ");
  WebSerial.print(curr_t_ctrl);
  WebSerial.print("; ");
  WebSerial.print(curr_t_mag);
  WebSerial.print("; ");
  WebSerial.print(curr_t_pod_d);
  WebSerial.print("; ");
  WebSerial.print(curr_t_nag_d);
  WebSerial.print("; "); 
  WebSerial.print(curr_t_ctrl_d);
  WebSerial.print("; "); 
  WebSerial.print(curr_t_mag_d);
  WebSerial.print("; ");
  WebSerial.print(goal_t);
  WebSerial.print("; ");
  WebSerial.print(Status);
  WebSerial.print("; ");
  WebSerial.print(Status_leak);
  WebSerial.println("; ");
}

//Функция приема сообщений (нуждается в существенном сокращении и изменении)
void webserialget(uint8_t *data, size_t len){
  unsigned long val = 0;
  //WebSerial.println("Received Data...");
  //String d = "";
  char key = char(data[0]);
  for(int i=1; i < len; i++){
    //d += char(data[i]);
    val = val + pow (10, (i-1)) * (char(data[i]) - '0');
  }
  //WebSerial.println(d);
  executer(key, val);
}

//Функция выполнения полученных команд. Исполняет полученные через serial команды. На вход принимает ключ и число. Добавить отключение датчиков протечки и отключение serial.
bool executer(char key, unsigned long val) 
{
  switch (key) {
    case 'g': //Целевая температура
      goal_t = val;
      Setpoint = val;
      pref.begin("heat_param", false);
      pref.putULong("goal_t", goal_t);
      pref.end();
      return true;
    break;

    case 's': //Старт или стоп нагрев (присваивает статусу принимаемое значение)
      if (val < 4) {
        Status = val;
        return true;
      }
    break;

    case 'h': //Гистерезис, вводить в 1000 раз большее значение (это позволяет вводить от 0,001 до 4 млн)
      hyst_t = val / 1000;
      pref.begin("heat_param", false);
      pref.putFloat("hyst_t", hyst_t);
      pref.end();
      return true;
    break;

    case 'k': //Коэффициент обратной связи, вводить в 1000 раз большее значение (это позволяет вводить от 0,001 до 4 млн)
      k_t = val / 1000;
      pref.begin("heat_param", false);
      pref.putFloat("k_t", k_t);
      pref.end();
      return true;
    break;

    case 'r': //Время изменения состояния реле в мс
      if (val > 100) {
        time_relay = val;
        pref.begin("heat_param", false);
        pref.putULong("time_relay", time_relay);
        pref.end();
        return true;
      }
    break;

    case 'l': //Вкл/выкл serial
      Serial_st = val;
      if (Serial_st or WebSerial_st) { //Если хотябы один Serial включен, то принятые значения можно записать в память. В ином случае не записывать, пусть при перезагрузке сбросятся.
        pref.begin("heat_param", false);
        //pref.putULong("Serial_st", Serial_st); //Выбрать подходящий тип данных
        pref.end();
        return true;
      }
    break;

    case 'w': //Вкл/выкл webserial
      WebSerial_st = val;
      if (Serial_st or WebSerial_st) { //Если хотябы один Serial включен, то принятые значения можно записать в память. В ином случае не записывать, пусть при перезагрузке сбросятся.
        pref.begin("heat_param", false);
        //pref.putULong("WebSerial_st", WebSerial_st); //Выбрать подходящий тип данных
        pref.end();
        return true;
      }
    break;

    case 'i': //Выводит общую информацию о системе в serial и webserial
      if (WebSerial_st) {infowebserialput();}
      if (Serial_st) {infoserialput();}
      if (trigger) {
        char buf[256];
        //Временный вариант для передачи информации пока не встроено в интерфейс
        udp_data.beginPacket(serverip, debugport); //Ввод ip и порта
        sprintf(buf, "Целевая температура: %.2d | Статус: %d", goal_t, relay_status);
        udp_data.printf(buf); //Запись пакета с данными на RPi
        udp_data.endPacket(); //Завершение пакета и его отправка
      }
    break;


    case 'o': //Старт или стоп нагрев (присваивает статусу принимаемое значение)
      //if (val == (0 || 1 || 2)) { //Такую же штуку прописать в других коэффициентах
      relay_status = val;
      pref.begin("heat_param", false);
      pref.putULong("relay_status", relay_status);
      pref.end();
      return true;
      //}
    break;

  }
  return false;
}

//Функция получения и сглаживания данных термопар (на таймере). Имеет смысл после переработки названий термопар перевести в цикл
void tcget31855() {
  if (not tc_nag.read()) {
    curr_t_nag = meas_rate * tc_nag.getTemperature() + ((1 - meas_rate) * curr_t_nag);
    curr_t_nag_d = meas_rate * tc_nag.getInternal() + ((1 - meas_rate) * curr_t_nag_d);
  } else {Status = Err;}
  if (not tc_pod.read()) {
    curr_t_pod = meas_rate * tc_pod.getTemperature() + ((1 - meas_rate) * curr_t_pod);
    curr_t_pod_d = meas_rate * tc_pod.getInternal() + ((1 - meas_rate) * curr_t_pod_d);
  } else {Status = Err;}
  if (not tc_ctrl.read()) {
    curr_t_ctrl = meas_rate * tc_ctrl.getTemperature() + ((1 - meas_rate) * curr_t_ctrl);
    curr_t_ctrl_d = meas_rate * tc_ctrl.getInternal() + ((1 - meas_rate) * curr_t_ctrl_d);
  } else {Status = Err;}
  if (not tc_mag.read()) {
    curr_t_mag = meas_rate * tc_mag.getTemperature() + ((1 - meas_rate) * curr_t_mag);
    curr_t_mag_d = meas_rate * tc_mag.getInternal() + ((1 - meas_rate) * curr_t_mag_d);
  } else {Status = Err;}
  /*
  Serial.print(tc_nag.getInternal());
  Serial.println(tc_nag.getTemperature()); 
  tc_pod.read();
  Serial.print(tc_pod.getInternal());
  Serial.println(tc_pod.getTemperature());
  */
}

//Функция вывода полной информации в Serial
void infoserialput() {
  Serial.println("=====Текущая информация=====");
  Serial.print("Текущая температура подложки: "); Serial.println(curr_t_pod);
  Serial.print("Текущая температура нагревателя: "); Serial.println(curr_t_nag);
  Serial.print("Текущая температура контрольной термопары: "); Serial.println(curr_t_ctrl);
  Serial.print("Текущая температура магнетрона: "); Serial.println(curr_t_mag);
  Serial.print("Текущая температура датчика термопары для подложки: "); Serial.println(curr_t_pod_d);
  Serial.print("Текущая температура датчика термопары для нагревателя: "); Serial.println(curr_t_nag_d);
  Serial.print("Текущая температура датчика контрольной термопары: "); Serial.println(curr_t_ctrl_d);
  Serial.print("Текущая температура датчика термопары магнетрона: "); Serial.println(curr_t_mag_d);
  Serial.print("Статус протечки: ");
  if (Status_leak == 0) {Serial.println("протечка отсутствует");} else 
    {
      Serial.print("протечка обнаружена ");
      Serial.print(Status_leak);
      if (Status_leak == 1) {Serial.println(" датчиком");} else {Serial.println(" датчиками");}
    }
  Serial.println("=====Конец текущей информации=====");
  Serial.println("=====Общая информация=====");
  Serial.print("Целевая температура: "); Serial.println(goal_t);
  Serial.print("Статус: "); Serial.println(relay_status);
  Serial.print("Гистерезис: "); Serial.println(hyst_t);
  Serial.print("Коэффициент обратной связи: "); Serial.println(k_t);
  Serial.print("Время изменения состояния реле в мс: "); Serial.println(time_relay);
  if (Serial_st) {Serial.println("Serial включен");} else {Serial.println("Serial выключен");}
  if (WebSerial_st) {Serial.println("WebSerial включен");} else {Serial.println("Serial выключен");}
  Serial.println("=====Конец общей информации=====");
  Serial.println("=====Справочная информация=====");
  Serial.println("Система автоматического управления нагревом by zertet");
  Serial.println("Управление системой происходит посредством комманд. Каждая команда состоит из ключа и числа.");
  Serial.println("Например, команда 'g260' установит целевую температуру в 260 градусов цельсия.");
  Serial.println("g - целевая температура нагрева");
  Serial.println("s - статус нагрева. 0 - выкл, 1 - вкл, 2 - готово, 3 - ошибка");
  Serial.println("h - гистерезис в градусах цельсия. Вводить значение, умноженное на 1000.");
  Serial.println("k - коэффициент обратной связи. Вводить значение, умноженное на 1000.");
  Serial.println("r - скорость срабатывания реле в мс");
  Serial.println("l - вкл/выкл Serial");
  Serial.println("w - вкл/выкл WebSerial. Требует перезагрузку.");
  Serial.println("i - вывести эту справку");
  Serial.println("Таким образом для включения WebSerial нужно ввести w1, для получения справки i1, для установки гистерезиса в значение 1 градус цельсия h1000.");
  Serial.println("=====Конец справочной информации=====");
  //Serial.println(test);
}

//Функция вывода полной информации в WebSerial
//Пока нет для магнеторона, ибо WebSerial не используется
void infowebserialput() {
  WebSerial.println("=====Текущая информация=====");
  WebSerial.print("Текущая температура подложки: "); WebSerial.println(curr_t_pod);
  WebSerial.print("Текущая температура нагревателя: "); WebSerial.println(curr_t_nag);
  WebSerial.print("Текущая температура контрольной термопары: "); WebSerial.println(curr_t_ctrl);
  WebSerial.print("Текущая температура датчика термопары для подложки: "); WebSerial.println(curr_t_pod_d);
  WebSerial.print("Текущая температура датчика термопары для нагревателя: "); WebSerial.println(curr_t_nag_d);
  WebSerial.print("Текущая температура датчика контрольной термопары: "); WebSerial.println(curr_t_ctrl_d);
  WebSerial.print("Статус протечки: ");
  if (Status_leak == 0) {WebSerial.println("протечка отсутствует");} else 
    {
      WebSerial.print("протечка обнаружена ");
      WebSerial.print(Status_leak);
      if (Status_leak == 1) {WebSerial.println(" датчиком");} else {WebSerial.println(" датчиками");}
    }
  WebSerial.println("=====Конец текущей информации=====");
  WebSerial.println("=====Общая информация=====");
  WebSerial.print("Целевая температура: "); WebSerial.println(goal_t);
  WebSerial.print("Статус: "); WebSerial.println(Status);
  WebSerial.print("Гистерезис: "); WebSerial.println(hyst_t);
  WebSerial.print("Коэффициент обратной связи: "); WebSerial.println(k_t);
  WebSerial.print("Время изменения состояния реле в мс: "); WebSerial.println(time_relay);
  if (Serial_st) {WebSerial.println("Serial включен");} else {WebSerial.println("Serial выключен");}
  if (WebSerial_st) {WebSerial.println("WebSerial включен");} else {WebSerial.println("Serial выключен");}
  WebSerial.println("=====Конец общей информации=====");
  WebSerial.println("=====Справочная информация=====");
  WebSerial.println("Система автоматического управления нагревом by zertet");
  WebSerial.println("Управление системой происходит посредством комманд. Каждая команда состоит из ключа и числа.");
  WebSerial.println("Например, команда 'g260' установит целевую температуру в 260 градусов цельсия.");
  WebSerial.println("g - целевая температура нагрева");
  WebSerial.println("s - статус нагрева. 0 - выкл, 1 - вкл, 2 - готово, 3 - ошибка");
  WebSerial.println("h - гистерезис в градусах цельсия. Вводить значение, умноженное на 1000.");
  WebSerial.println("k - коэффициент обратной связи. Вводить значение, умноженное на 1000.");
  WebSerial.println("r - скорость срабатывания реле в мс");
  WebSerial.println("l - вкл/выкл Serial");
  WebSerial.println("w - вкл/выкл WebSerial. Требует перезагрузку.");
  WebSerial.println("i - вывести эту справку");
  WebSerial.println("Таким образом для включения WebSerial нужно ввести w1, для получения справки i1, для установки гистерезиса в значение 1 градус цельсия h1000.");
  WebSerial.println("=====Конец справочной информации=====");
}

//Функция отправки уведомлений при детектировании протечки
int leak_notification () {
  WiFiClient client;
  HTTPClient http;
  // Your Domain name with URL path or IP address with path
  http.begin(client, "http://ntfy.mt11.su:8081/vup_emergency");
  // Specify content-type header
  http.addHeader("Title", "Система ВУП-11М");
  http.addHeader("Priority", "max");
  // Data to send with HTTP POST
  String httpRequestData = "Обнаружена протечка!";
  // Send HTTP POST request
  int httpResponseCode = http.POST(httpRequestData);
  Serial.print("HTTP Response code: ");
  Serial.println(httpResponseCode);
  http.end();
  return httpResponseCode;
}

//Функция отправки уведомлений при детектировании экстремальных температур
void temp_notification () {
  WiFiClient client;
  HTTPClient http;
  // Your Domain name with URL path or IP address with path
  http.begin(client, "http://ntfy.mt11.su:8081/vup_emergency");
  // Specify content-type header
  http.addHeader("Title", "Система ВУП-11М");
  http.addHeader("Priority", "max");
  // Data to send with HTTP POST
  String httpRequestData = "Сheck the temperature!";
  // Send HTTP POST request
  int httpResponseCode = http.POST(httpRequestData);
  Serial.print("HTTP Response code: ");
  Serial.println(httpResponseCode);
  http.end();
}


void setup() 
{
  //Задание Serial
  Serial.begin(115200);
  Serial.setTimeout(1); //Установка таймаута для парсинга принимаемых значений (потом мб уменьшить или увеличить)
  delay (2000);

  PIDregulator.Reset();
  PIDregulator.SetOutputLimits(0, 255);
     
  Setpoint = goal_t;
  PIDregulator.SetTunings(Kp, Ki, Kd);
  PIDregulator.SetSampleTimeUs(2000000);  
  PIDregulator.SetMode(PIDregulator.Control::automatic);

  Serial.println("ESP32 начинает включаться");
  
  //Задание WiFi
  if (WebSerial_st) {
    WiFi.mode(WIFI_STA); //Выбираем режим WiFi
    WiFi.begin(ssid, password); //Логинемся
    if (WiFi.waitForConnectResult() != WL_CONNECTED) { //Проверяем, успешна ли авторизация
        Serial.printf("WiFi Failed!\n");
        WebSerial_st = Off; //Выключаем вебсериал чтобы не было ошибок
        return;
    }
    else {
    udp.begin(localPort);
    Serial.printf("UDP Client : %s:%i \n", WiFi.localIP().toString().c_str(), localPort); //Информация о IP Arduino
    }
  }

  if (WebSerial_st) {
    Serial.print("IP Address: "); //Выводим в Serial свой ip
    Serial.println(WiFi.localIP());
    if (Serial_st) {Serial.println("WebSerial доступен по <IP Address>/webserial в браузере");}
    WebSerial.begin(&server); //Стартуем WEBSerial передавая в него параметром объект класса AsyncWebServer
    WebSerial.onMessage(webserialget); //Задаем функцию которая будет вызываться при получении сообщения
    server.begin(); //Запускаем веб сервер 
  }

  //Задание SPI для датчиков термопар
  SPI.begin();
  tc_pod.begin();
  tc_nag.begin();
  tc_ctrl.begin();
  tc_mag.begin();

  //Дополнительные 4 бита задержки для улучшения сигнала
  tc_pod.setSWSPIdelay(4);
  tc_nag.setSWSPIdelay(4);
  tc_ctrl.setSWSPIdelay(4);
  tc_mag.setSWSPIdelay(4);

  //Задание и чтение энергонезависимой памяти
  pref.begin("heat_param", false); //???
  goal_t = pref.getULong("goal_t", goal_t);
  hyst_t = pref.getFloat("hyst_t", hyst_t);
  k_t = pref.getFloat("k_t", k_t);
  time_relay = pref.getULong("time_relay", time_relay);
  relay_status = pref.getULong("relay_status", relay_status);
  //Serial_st = pref.getULong("Serial_st", Serial_st); //Выбрать подходящий тип данных
  //WebSerial_st = pref.getULong("WebSerial_st", WebSerial_st); //Выбрать подходящий тип данных
  pref.end();

  //Задание пинов переферии
  //pinMode(HeatOn_s_pin, INPUT);
  //pinMode(HeatOff_s_pin, INPUT);
  pinMode(Done_pin, OUTPUT);
  //pinMode(Relay_pin, OUTPUT);

  //На всякий случай. Потом разобраться с бибой гувера и мб убрать 
  pinMode(Solid_relay_pin, OUTPUT);
  digitalWrite(Solid_relay_pin, LOW);

  //Настройка регулятора
  regulator.setpoint = goal_t; //Установка целевой температуры
  regulator.hysteresis = hyst_t; //Установка значения гистерезиса
  regulator.k = k_t;

  //Задание таймеров
  tmr_print.start(time_print);
  tmr_serial_put.start(time_serial);
  tmr_UDP_put.start(time_UDP);
  tmr_tc_get.start(time_tc);
  

  Serial.println("Проверка датчиков");
  for (int i = 0; i < (meas_count * 2); i++) { //Цикл первичных измерений. Нужен, чтобы наработать значения для скользящего среднего.
    delay(time_tc + 1); //Задержка опроса датчиков
    tcget31855();
  }

  //Первичный расчёт параметров для регуляции реле
  Input = curr_t_ctrl;
  //Input = curr_t_pod;
  PIDregulator.Compute();
  
  if (Output != 0 && Output != 255) {period = max((20 / (Output/255)), (20 / (1 - (Output/255))));}
  else {period = time_tc;}
  Solid_relay.setPeriod(period);
  Solid_relay.setPWM(Output);

  if (Status == Err) {Serial.println("Ошибка! Температура не доступна!");} //???

  if (Serial_st) {infoserialput();} //Вывод полной информации в serial    

  tuner.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner.SetEmergencyStop(tempLimit);

  tuner5T.Configure(inputSpan, outputSpan, outputStart, outputStep, testTimeSec, settleTimeSec, samples);
  tuner5T.SetEmergencyStop(tempLimit);

}

void loop() 
{
//Часть ответственная за получение информации от RPi
//Тестовый варинат
int packetSize = udp.parsePacket();
if (packetSize) {
  memset(packetBuffer, 0, sizeof(packetBuffer)); //Обнуляем буфер на всякий случай. Мб убрать
  int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);  // читаем не более 15 байт
  if (len > 0) {
    packetBuffer[len] = '\0';  // ← теперь это безопасная строка
  }
  if ((strcmp(previousBuffer, packetBuffer) != 0) || (packetBuffer[0] == 'i')) {
    trigger = true;
    serialget();
  }
  
  //Serial.print("Received packet from : "); Serial.println(udp.remoteIP());
  //udp.read(packetBuffer, 255);
  //Serial.printf("Data : %s\n", packetBuffer);
  //Serial.println();
}

/*

  tuner.softPwm(Solid_relay_pin, Input, Output, 0, outputSpan, 1); //Вроде ничего не сделает
  tuner5T.softPwm(Solid_relay_pin, Input, Output, 0, outputSpan, 1); //Вроде ничего не сделает

  switch (tuner.Run()) {
    case tuner.sample: // active once per sample during test
      Input = curr_t_ctrl;
      break;

    case tuner.tunings: // active just once when sTune is done
      //Output = 0;
      tuner.SetTuningMethod(tuner.TuningMethod::DampedOsc_PID);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::NoOvershoot_PID);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::CohenCoon_PID);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::Mixed_PID);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::ZN_PI);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::DampedOsc_PI);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::NoOvershoot_PI);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::CohenCoon_PI);
      tuner.printTunings();
      tuner.SetTuningMethod(tuner.TuningMethod::Mixed_PI);
      tuner.printTunings();
      break;
  }

  switch (tuner5T.Run()) {
    case tuner5T.sample: // active once per sample during test
      Input = curr_t_ctrl;
      break;

    case tuner5T.tunings: // active just once when sTune is done
      //Output = 0;
      tuner5T.SetTuningMethod(tuner5T.TuningMethod::DampedOsc_PID);
      tuner5T.printTunings();
      tuner5T.SetTuningMethod(tuner5T.TuningMethod::NoOvershoot_PID);
      tuner5T.printTunings();
      tuner5T.SetTuningMethod(tuner5T.TuningMethod::CohenCoon_PID);
      tuner5T.printTunings();
      tuner5T.SetTuningMethod(tuner5T.TuningMethod::Mixed_PID);
      tuner5T.printTunings();
      tuner5T.SetTuningMethod(tuner5T.TuningMethod::ZN_PI);
      tuner5T.printTunings();
      tuner5T.SetTuningMethod(tuner5T.TuningMethod::DampedOsc_PI);
      tuner5T.printTunings();
      tuner5T.SetTuningMethod(tuner5T.TuningMethod::NoOvershoot_PI);
      tuner5T.printTunings();
      tuner5T.SetTuningMethod(tuner5T.TuningMethod::CohenCoon_PI);
      tuner5T.printTunings();
      tuner5T.SetTuningMethod(tuner5T.TuningMethod::Mixed_PI);
      tuner5T.printTunings();
      break;
  }

*/
  float ComputedOutput;
  char buf[256]; //Буфер для передачи данных

  //Перевод милисекунд. Может быть есть готовая функция для первода, но и так сойдёт
  int hours = millis() / (60 * 60 * 1000);
  int minuts = millis() / (60 * 1000) - hours * 60;
  int seconds = millis() / 1000 - hours * 3600 - minuts * 60;

  //Расчёт параметров для регуляции реле
  if (relay_status == 2) {
    if (PIDregulator.Compute()) {
      Input = curr_t_ctrl; //Стоит изменть переменную температуры при надобности
      //Input = curr_t_pod;
      ComputedOutput = Output;
      
      if (ComputedOutput != 0 && ComputedOutput != 255) {period = max((20 / (ComputedOutput/255)), (20 / (1 - (ComputedOutput/255))));}
      else {period = time_tc;}
  
      Serial.printf("| %.2d:%.2d:%.2d | %.2f %.2f | %.2f %.2f |  %llu | %llu | %.3f | %llu |",
        hours, minuts, seconds, curr_t_pod, curr_t_pod_d, curr_t_ctrl, curr_t_ctrl_d, 
        goal_t, relay_status, ComputedOutput, period);

      Solid_relay.setPeriod(period);
      Solid_relay.setPWM(ComputedOutput);
    }  
     
    if (relay_status == 0) { //Проверка на возможность использование реле
      digitalWrite(Solid_relay_pin, LOW); 
    }
    else if (relay_status == 1) {
      digitalWrite(Solid_relay_pin, HIGH); 
    }
    else if (relay_status == (2 || 3)) { //При 3 вычислений нет, остаётся последняя вычисленная скважность и период
      Solid_relay.tick(); //Функция, которая отвечает за включение и выключение реле в зависимотсти от скважности и периода
    }

  }
  
  
  
  serialget(); //Приём сигналов по сериал

  if (tmr_serial_put.ready(1)) {
    Leak_test();
    //if (Serial_st) {serialput();} //Вывод в serial
    //if (WebSerial_st) {webserialput();} //Вывод в webserial, отключен за ненадобностью
  }

  if (tmr_tc_get.ready(1)) {tcget31855();} //Получение данных от термопар

  if (tmr_UDP_put.ready(1)) { //Передача информации на RPi
    udp_data.beginPacket(serverip, serverport); //Ввод ip и порта
    //Временный вариант для передачи показаний пока не встроено в интерфейс
    //sprintf(buf, "| %.2d:%.2d:%.2d | %.2f %.2f | %.2f %.2f | %.2f %.2f | %.2f %.2f |", hours, minuts, seconds, curr_t_pod, curr_t_pod_d, curr_t_nag, curr_t_nag_d, curr_t_ctrl, curr_t_ctrl_d, curr_t_mag, curr_t_mag_d);
    sprintf(buf, "| %.2d:%.2d:%.2d | %.2f %.2f | %.2f %.2f | %.3f |", hours, minuts, seconds, curr_t_pod, curr_t_pod_d, curr_t_ctrl, curr_t_ctrl_d, ComputedOutput);
    udp_data.printf(buf); //Запись пакета с данными на RPi
    udp_data.endPacket(); //Завершение пакета и его отправка
  }
}