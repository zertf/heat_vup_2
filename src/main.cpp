
//Прога регулятора нагрева

//Библиотеки
#include <Arduino.h> //Библиотека стандартных команд arduino
//#include "PWMrelay.h" //Библиотека ШИМ сигнала
//#include "GyverPID.h" //Библиотека ПИД регулирования
#include "GyverRelay.h" //Библиотека релейного регулирования
//#include <EasyButton.h> //Библиотека приема сигналов с кнопок
#include <LiquidCrystal_I2C.h> //Библиотека для i2c дисплеев
#include <Preferences.h> //Библиотека для работы с энергонезависимой памятью конкретно в ESP32
#include "MAX31855.h" //Установлена библиотека Роба Тилларта MAX31855_RT
//#include <WiFi.h> //Библиотека для работы с WiFi
#include <AsyncTCP.h>           //Какая-то служебная библиотека для WebSerialLite.h
#include <ESPAsyncWebServer.h>  //Какая-то служебная библиотека для WebSerialLite.h
#include <WebSerialLite.h>      //Библиотека Serial по WEB (облегченная версия, то что нужно) 


//Месточисления для статуса
#define Off 0
#define On 1
#define Done 2
#define Err 3

//Объявление констант
//Константы пинов
const byte HeatOn_s_pin = 33; //Пин для включенного положения переключателя (пока что не используется)
const byte HeatOff_s_pin = 25; //Пин для выключенного положения переключателя (пока что не используется)
const byte Done_pin = 26; //Светодиод готовности (с появлением экрана стал почти не нужен) (пока что не используется)
const byte Relay_pin = 32; //Управление реле нагрева
const byte Leak_pin1 = 34; //Пины датчиков протечки, подтянуты вверх, чтобы сработать на землю (пока что не используется)
const byte Leak_pin2 = 35;
const byte Leak_pin3 = 27;
const byte tc_pod_CS = 4; //Пины CS (SS) сигнала активации датчиков термопары
const byte tc_nag_CS = 16;
//18 и 19 пины заняты под SPI

const byte meas_count = 5; //Окно скользящего среднего для значений температуры (мб логично что это должно быть в пределах от 3 до 8)
const float meas_rate = (2 / (meas_count + 1)); //Используется для вычисления скользящего среднего
const char* ssid = "Minternet"; //SSID сети
const char* password = "dodgezoo"; //WiFi Password

//Объявим переменные
byte Status = Off; //Переменная статуса нагрева; 0 - выкл, 1 - вкл, 2 - достигло цели, поддержание температуры
byte Status_leak = 0; //Переменная статуса протечек
//byte i = 0;//Переменная счётчика (для отладки)
unsigned long time_print = 3000; //Время таймера выводы на экран
unsigned long time_serial = 5000; //Время таймера вывода информации по serial
bool Serial_st = On; //Включает - выключает Serial
bool WebSerial_st = On; //Включает - выключает WebSerial
bool Display_st = On; //Включает - выключает обновление дисплея

//Переменные связанные с нагревом
float hyst_t = 5; //Гистерезис температуры                  (изменить тип данных на тот, который подойдёт для установки)
unsigned long goal_t = 400; //Целевая температура
float curr_t_pod = 10; //Текущая температура подложки
float curr_t_nag = 10; //Текущая температура нагревателя
float curr_t_pod_d = 10; //Текущая температура датчика термопары подложки
float curr_t_nag_d = 10; //Текущая температура датчика термопары нагревателя
unsigned long time_relay = 1000; //Время изменения состояния реле в мс
unsigned long time_tc = 1000; //Время таймера опроса датчиков термопары (не может быть меньше 100 мс)
float k_t = 0.5; //Коэффициент обратной связи (нужно подобрать)
unsigned long tmr_relay = 0; //Таймер изменения состояния реле (на millis, ручной)

//Задание классов
GyverRelay regulator(REVERSE); //Задание класса регулятора
LiquidCrystal_I2C lcd = LiquidCrystal_I2C(0x27, 20, 4); //Задание класса экрана
Preferences pref; //Задание класса энергонезависимой памяти
MAX31855 tc_pod(tc_pod_CS, &SPI); //Задание класса для датчика температуры подложки
//MAX31855 tc_pod(17, 19, 18);
MAX31855 tc_nag(tc_nag_CS, &SPI); //Задание класса для датчика температуры нагревателя
AsyncWebServer server(80); //Задание класса вебсервера на 80 порту

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

//Функция проверки статуса (требует переработки)
void Status_check() {
  if (curr_t_pod > goal_t + 30) {
    Status = Err;
    digitalWrite(Done_pin, LOW);
  } else if (curr_t_pod > goal_t - 5) {
    Status = Done;
    digitalWrite(Done_pin, HIGH);
  } else {
    Status = On;
    digitalWrite(Done_pin, LOW);
  }

}

//Функция проверки пинов протечки
void Leak_test() {
  Status_leak = 3 - digitalRead(Leak_pin1) - digitalRead(Leak_pin2) - digitalRead(Leak_pin3); //Если 1 датчик зафиксирует протечку, то статус станет 1
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
    //unsigned long ms = millis();
    char key = Serial.read();
    unsigned long val = Serial.parseInt();
    switch (key) {
      case 'g': //Целевая температура
        goal_t = val;
        pref.begin("heat_param", false);
        pref.putULong("goal_t", goal_t);
        pref.end();
      break;
      case 's': //Старт или стоп нагрев (присваивает статусу принимаемое значение)
        if (val < 4) {Status = val;}
      break;
      case 'h': //Гистерезис, вводить в 1000 раз большее значение (это позволяет вводить от 0,001 до 4 млн)
        hyst_t = val / 1000;
        pref.begin("heat_param", false);
        //pref.putFloat("hyst_t", hyst_t);
        pref.end();
      break;
      case 'k': //Коэффициент обратной связи, вводить в 1000 раз большее значение (это позволяет вводить от 0,001 до 4 млн)
        k_t = val / 1000;
        pref.begin("heat_param", false);
        //pref.putFloat("k_t", k_t);
        pref.end();
      break;
      case 'r': //Время изменения состояния реле в мс
        if (val > 50) {
          time_relay = val;
          pref.begin("heat_param", false);
          //pref.putULong("time_relay", time_relay);
          pref.end();
        }
      break;

    }
    //Serial.println(millis() - ms);
  }
}

//Функция отправки сигналов по Serial (на таймере)
void serialput() {
  Serial.print(curr_t_pod, 0);
  Serial.print("; ");
  Serial.print(curr_t_nag, 0);
  Serial.print("; ");
  Serial.print(curr_t_pod_d, 0);
  Serial.print("; ");
  Serial.print(curr_t_nag_d, 0);
  Serial.print("; ");
  Serial.print(goal_t);
  Serial.print("; ");
  Serial.print(Status);
  Serial.println("; ");
}

//Функция отправки сигналов по WebSerial (на таймере)
void webserialput() {
  WebSerial.print(curr_t_pod);
  WebSerial.print("; ");
  WebSerial.print(curr_t_nag);
  WebSerial.print("; ");
  WebSerial.print(curr_t_pod_d);
  WebSerial.print("; ");
  WebSerial.print(curr_t_nag_d);
  WebSerial.print("; ");
  WebSerial.print(goal_t);
  WebSerial.print("; ");
  WebSerial.print(Status);
  WebSerial.println("; ");
}

//Функция приема сообщений (нуждается в существенном сокращении)
void recvMsg(uint8_t *data, size_t len){
  WebSerial.println("Received Data...");
  String d = "";
  for(int i=0; i < len; i++){
    d += char(data[i]);
  }
  WebSerial.println(d);
}

//Функция получения данных термопар (на таймере)
void tcget() {
  byte S_tc_pod = tc_pod.read();
  //byte S_tc_nag = tc_nag.read();
  if (S_tc_pod) { //S_tc_pod + S_tc_nag
    Status = Err; //Добавить сюда ещё инфы для отладки (желательно, чтобы в Serial и на экран транслировалось)
  } else {
    //curr_t_pod = meas_rate * tc_pod.getTemperature() + ((1 - meas_rate) * curr_t_pod); //Вычисление скользящего среднего
    //curr_t_pod = tc_pod.getTemperature();
    Serial.println(tc_pod.getTemperature());
    //curr_t_nag = meas_rate * tc_nag.getTemperature() + ((1 - meas_rate) * curr_t_nag);
    //curr_t_pod_d = meas_rate * tc_pod.getInternal() + ((1 - meas_rate) * curr_t_pod_d);
    //curr_t_pod_d = tc_pod.getInternal();
    Serial.println(tc_pod.getInternal());
    Serial.println(tc_pod.getRawData(), BIN);
    //curr_t_nag_d = meas_rate * tc_nag.getInternal() + ((1 - meas_rate) * curr_t_nag_d);
  }
}

void setup() {
  //Задание Serial
  Serial.begin(115200);
  Serial.setTimeout(1); //Установка таумаута для парсинга принимаемых значений (потом мб уменьшить или увеличить)
  delay (150);
  Serial.println("ESP32 начинает включаться");

  //Задание WiFi (ничего не понятно, разобраться!)
  WiFi.mode(WIFI_STA); //Выбираем режим WiFi
  WiFi.begin(ssid, password); //Логинемся
  if (WiFi.waitForConnectResult() != WL_CONNECTED) { //Проверяем, успешна ли авторизация
      Serial.printf("WiFi Failed!\n");
      return;
  }
  Serial.print("IP Address: "); //Выводим в Serial свой ip
  Serial.println(WiFi.localIP());
  // WebSerial доступен по "<IP Address>/webserial" в браузере
  WebSerial.begin(&server); //Стартуем WEBSerial передавая в него параметром объект класса AsyncWebServer
  WebSerial.onMessage(recvMsg); //Задаем функцию которая будет вызываться при получении сообщения
  server.begin(); //Запускаем веб сервер 

  //Задание SPI для датчиков термопар
  SPI.begin();
  tc_pod.begin();
  tc_pod.setSPIspeed(9000);
  //tc_nag.begin();

  //Задание и чтение энергонезависимой памяти
  pref.begin("heat_param", false);
  goal_t = pref.getULong("goal_t", goal_t);
  hyst_t = pref.getFloat("hyst_t", hyst_t);
  k_t = pref.getFloat("k_t", k_t);
  time_relay = pref.getULong("time_relay", time_relay);
  pref.end();

  //Задание пинов переферии
  //pinMode(HeatOn_s_pin, INPUT);
  //pinMode(HeatOff_s_pin, INPUT);
  pinMode(Done_pin, OUTPUT);
  pinMode(Relay_pin, OUTPUT);
  pinMode(Leak_pin1, INPUT_PULLUP);
  pinMode(Leak_pin2, INPUT_PULLUP);
  pinMode(Leak_pin3, INPUT_PULLUP);

  //Настройка регулятора
  regulator.setpoint = goal_t; //Установка целевой температуры
  regulator.hysteresis = hyst_t; //Установка значения гистерезиса
  regulator.k = k_t;

  //Задание таймеров
  tmr_print.start(time_print);
  tmr_serial_put.start(time_serial);
  tmr_tc_get.start(time_tc);
  //tmr_1.start(1000); //Таймер отладки

  Serial.println("Проверка датчиков");
  for (int i = 0; i < (meas_count * 2); i++) { //Цикл первичных измерений. Нужен, чтобы наработать значения для скользящего среднего.
    delay(time_tc); //Задержка опроса датчиков
    tcget();
  }
  if (Status == Err) {Serial.println("Ошибка! Температура не доступна!");}

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
  }
}

void loop() {
  //Leak_test();
  //Status_check();
  regulator_check(); //Срабатывает по таймеру, чтобы передать температуру и изменить состояние реле
  if (tmr_print.ready(1) and Display_st) {print_display();} //Вывод информации на lcd дисплей
  serialget();
  if (tmr_serial_put.ready(1)) {
    //if (Serial_st) {serialput();}
    //if (WebSerial_st) {webserialput();}
  }
  if (tmr_tc_get.ready(1)) {tcget();}
}
