//Прога регулятора нагрева

//Библиотеки
#include <Arduino.h> //Библиотека стандартных команд arduino
//#include "PWMrelay.h" //Библиотека ШИМ сигнала
//#include "GyverPID.h" //Библиотека ПИД регулирования
#include "GyverRelay.h" //Библиотека релейного регулирования
//#include <EasyButton.h> //Библиотека приема сигналов с кнопок
#include <LiquidCrystal_I2C.h> //Библиотека для i2c дисплеев
#include <Preferences.h> //Библиотека для работы с энергонезависимой памятью конкретно в ESP32
#include "MAX31855.h" //Библиотека Роба Тилларта MAX31855_RT
#include "MAX6675.h" //Библиотека Роба Тилларта MAX6675
//#include <WiFi.h> //Библиотека для работы с WiFi
#include <HTTPClient.h> //Библиотека для отправки post запроса на ntfy
#include <AsyncTCP.h>           //Какая-то служебная библиотека для WebSerialLite.h
#include <ESPAsyncWebServer.h>  //Какая-то служебная библиотека для WebSerialLite.h
#include <WebSerialLite.h>      //Библиотека Serial по WEB (облегченная версия, то что нужно)
//#include <GyverMAX6675_SPI.h>   //Библиотека MAX6675
//#include "Adafruit_MAX31855.h"  //Библиотека MAX31855 от Adafruit

//Месточисления для статуса
#define Off 0
#define On 1
#define Done 2 //Не используется
#define Err 3

//Объявление констант
//Константы пинов
const byte HeatOn_s_pin = 33; //Пин для включенного положения переключателя (пока что не используется)
const byte HeatOff_s_pin = 25; //Пин для выключенного положения переключателя (пока что не используется)
const byte Done_pin = 26; //Светодиод готовности (с появлением экрана стал не нужен) (пока что не используется)
const byte Relay_pin = 32; //Управление реле нагрева
const byte Leak_pin1 = 13; //Пин первого датчика протечки
const byte Leak_pin2 = 14; //Пин второго датчика протечки
const byte Leak_pin3 = 27; //Пин третьего датчика протечки
const byte tc_pod_CS = 18; //Пин CS (SS) для подложкодержателя
const byte tc_nag_CS = 16; //Пин CS (SS) для нагревателя
const byte tc_ctrl_CS = 19; //Пин CS (SS) для контрольной термопары
const byte SPI_SO = 17; //Пин SPI
const byte SPI_SCK = 4; //Пин SPI
//21 и 22 пины заняты под i2c

const byte meas_count = 1; //Окно скользящего среднего для значений температуры (мб логично что это должно быть в пределах от 3 до 8, но если значения стабильны, то можно и 1). Чем больше, тем медленнее реакция системы, но лучше сглаживание.
const float meas_rate = (2.0 / (meas_count + 1.0)); //Используется для вычисления скользящего среднего
const char* ssid = "WiFi name"; //SSID сети
const char* password = "WiFi password"; //WiFi Password

//Объявим переменные
byte Status = Off; //Переменная статуса нагрева; 0 - выкл, 1 - вкл, 2 - достигло цели, поддержание температуры
byte Status_leak = 0; //Переменная статуса протечек; 0 - протечки нет, 1 - датчик зафиксировал протечку, 2 - 2 датчика зафиксировало протечку, 3 - 3 датчика
//byte i = 0;//Переменная счётчика (для отладки)
unsigned long time_print = 2000; //Время таймера выводы на экран
unsigned long time_serial = 3000; //Время таймера вывода информации по serial
bool Serial_st = On; //Включает - выключает Serial
bool WebSerial_st = On; //Включает - выключает WebSerial
bool Display_st = On; //Включает - выключает обновление дисплея
bool Notify_st = On; //Включает - выключает отправку уведомлений об экстренных ситуациях
bool Leak_notify_sended = 0; //Переменная для контроля отправки уведомления

//Переменные связанные с нагревом
float hyst_t = 5; //Гистерезис температуры
unsigned long goal_t = 400; //Целевая температура
float curr_t_pod = 10; //Текущая температура подложки
float curr_t_nag = 10; //Текущая температура нагревателя
float curr_t_ctrl = 10; //Текущая температура контрольной термопары
float curr_t_pod_d = 10; //Текущая температура датчика термопары подложки
float curr_t_nag_d = 10; //Текущая температура датчика термопары нагревателя
float curr_t_ctrl_d = 10; //Текущая температура датчика термопары нагревателя
unsigned long time_relay = 2000; //Время изменения состояния реле в мс. От него сильно зависит точность. Нужно подбирать оптимальное значение.
unsigned long time_tc = 230; //Время таймера опроса датчиков термопары (не может быть меньше 200 мс). Должно быть меньше time_relay.
float k_t = 0.5; //Коэффициент обратной связи (нужно подобрать)
unsigned long tmr_relay = 0; //Таймер изменения состояния реле (на millis, ручной)

//Задание классов
GyverRelay regulator(REVERSE); //Задание класса регулятора
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4); //Задание класса экрана
Preferences pref; //Задание класса энергонезависимой памяти
AsyncWebServer server(80); //Задание класса вебсервера на 80 порту
//GyverMAX6675_SPI<tc_pod_CS> tc_pod; //Задание класса для датчика температуры подложки
//Adafruit_MAX31855 tc_nag(SPI_SCK, tc_nag_CS, SPI_SO); //Задание класса датчика термопары нагревателя
//Adafruit_MAX31855 tc_pod(SPI_SCK, tc_pod_CS, SPI_SO); //Задание класса датчика термопары подложкодержателя
MAX31855 tc_nag(tc_nag_CS, SPI_SO, SPI_SCK); //Задание класса датчика термопары нагревателя
MAX31855 tc_pod(tc_pod_CS, SPI_SO, SPI_SCK); //Задание класса датчика термопары подложкодержателя
MAX6675 tc_ctrl(tc_ctrl_CS, SPI_SO, SPI_SCK); //Задание класса датчика контрольной термопары

//Создание класса Таймер с возможностью автоперезапуска и паузы
class Timer {
  /* Класс Таймера
  В слассе имеются следующие функции:
  1. Старт таймера (принимает на вход период)
  2. Старт таймера (без данных на входе просто перезагружает таймер)
  2. Стоп таймера (не реализовано так как не используется)
  3. Пауза
  4. Продолжение после паузы
  5. Продолжение после паузы с аргументом на услучай перезапуска
  6. Функция срабатывания (ничего не принимает на вход, возвращает кончился таймер или нет)
  7. Функция срабатывания (принимает на вход bool на случай если при срабатывании требуется сразу перезапустить таймер)
  8. Функция оставшегося времени (ничего не принимает, возвращает сколько времени осталось в мс)
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
Timer tmr_print; //Таймер на 1 секунду для вывода инфы на экран
Timer tmr_serial_put; //Таймер на 1 секунду для выводы инфы в сериал
Timer tmr_tc_get; //Таймер для опроса датчиков термопары

//Описание функций
void Status_check();          //Функция проверки статуса (требует переработки)
void Leak_test();             //Функция проверки пинов протечки. Проверяет пины на наличие сигнала и в соответствии с этим изменяет статус протечки.
void regulator_check();       //Функция управления реле (на таймере внутри функции)
void print_display();         //Функция вывода информации на lcd экран (на таймере)
void serialget();             //Функция приема сигналов по Serial
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
  if (Serial.available() > 1) {
    char key = Serial.read();
    unsigned long val = Serial.parseInt();
    executer(key, val);
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
  Serial.print(curr_t_pod_d, 0);
  Serial.print("; ");
  Serial.print(curr_t_nag_d, 0);
  Serial.print("; ");
  Serial.print(curr_t_ctrl_d, 0);
  Serial.print("; ");
  Serial.print(goal_t);
  Serial.print("; ");
  Serial.print(Status);
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
  WebSerial.print(curr_t_pod_d);
  WebSerial.print("; ");
  WebSerial.print(curr_t_nag_d);
  WebSerial.print("; "); 
  WebSerial.print(curr_t_ctrl_d);
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
bool executer(char key, unsigned long val) {
  switch (key) {
    case 'g': //Целевая температура
      goal_t = val;
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
    break;
  }
  return false;
}

//Функция получения и сглаживания данных термопар (на таймере)
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
    //curr_t_ctrl_d = meas_rate * tc_ctrl.getInternal() + ((1 - meas_rate) * curr_t_ctrl_d);
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
  Serial.print("Текущая температура датчика термопары для подложки: "); Serial.println(curr_t_pod_d);
  Serial.print("Текущая температура датчика термопары для нагревателя: "); Serial.println(curr_t_nag_d);
  Serial.print("Текущая температура датчика контрольной термопары: "); Serial.println(curr_t_ctrl_d);
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
  Serial.print("Статус: "); Serial.println(Status);
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
}

//Функция вывода полной информации в WebSerial
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

void setup() {
  //Задание Serial
  Serial.begin(115200);
  Serial.setTimeout(1); //Установка таймаута для парсинга принимаемых значений (потом мб уменьшить или увеличить)
  delay (150);
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

  //Задание и чтение энергонезависимой памяти
  pref.begin("heat_param", false);
  goal_t = pref.getULong("goal_t", goal_t);
  hyst_t = pref.getFloat("hyst_t", hyst_t);
  k_t = pref.getFloat("k_t", k_t);
  time_relay = pref.getULong("time_relay", time_relay);
  //Serial_st = pref.getULong("Serial_st", Serial_st); //Выбрать подходящий тип данных
  //WebSerial_st = pref.getULong("WebSerial_st", WebSerial_st); //Выбрать подходящий тип данных
  pref.end();

  //Задание пинов переферии
  //pinMode(HeatOn_s_pin, INPUT);
  //pinMode(HeatOff_s_pin, INPUT);
  pinMode(Done_pin, OUTPUT);
  pinMode(Relay_pin, OUTPUT);

  //Настройка регулятора
  regulator.setpoint = goal_t; //Установка целевой температуры
  regulator.hysteresis = hyst_t; //Установка значения гистерезиса
  regulator.k = k_t;

  //Задание таймеров
  tmr_print.start(time_print);
  tmr_serial_put.start(time_serial);
  tmr_tc_get.start(time_tc);

  Serial.println("Проверка датчиков");
  for (int i = 0; i < (meas_count * 2); i++) { //Цикл первичных измерений. Нужен, чтобы наработать значения для скользящего среднего.
    delay(time_tc + 1); //Задержка опроса датчиков
    tcget31855();
  }
  if (Status == Err) {Serial.println("Ошибка! Температура не доступна!");}

  if (WebSerial_st) {infowebserialput();}
  if (Serial_st) {infoserialput();}

  if (Display_st) { //Задание начального текста на экране
    lcd.begin();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("T pod");
    lcd.setCursor(0, 1);
    lcd.print("T nag");
    lcd.setCursor(0, 2);
    lcd.print("T goal ");
    lcd.print((goal_t));
    //lcd.setCursor(0, 3);
    lcd.setCursor(11, 2);
    lcd.print(" Heat Off");
    /*lcd.setCursor(0, 3);
    lcd.print("Leak: ");
    lcd.print("No");*/
  }
}

void loop() {
  //Status_check();
  if ((Status == On) or (Status == Done)) {regulator_check();} else {digitalWrite(Relay_pin, LOW);} //Срабатывает по таймеру и условию, чтобы передать температуру и изменить состояние реле
  if (tmr_print.ready(1) and Display_st) {print_display();} //Вывод информации на lcd дисплей
  serialget();
  if (tmr_serial_put.ready(1)) {
    Leak_test();
    if (Serial_st) {serialput();}
    if (WebSerial_st) {webserialput();}
  }
  if (tmr_tc_get.ready(1)) {tcget31855();}
}
