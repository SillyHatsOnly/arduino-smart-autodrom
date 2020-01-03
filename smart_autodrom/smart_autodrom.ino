//----------------------------------------------------------------------//
//  ПОЗИЦИИ ИГРОКА НА ПОЛЕ:
//----------------------------------------------------------------------//
#define READY_TO_START                            0                     //  Позиция готовности трассы к старту.
#define START_RACE                                1                     //  Позиция после пересечения датчика СТАРТ.
#define ESTAKADA_WAITING                          2                     //  Позиция нахождения на эстакаде.
#define GO_FROM_ESTAKADA                          3                     //  Позиция после эстакады.
#define STOP_BEFORE_PARKING                       4                     //  Позиция перед началом парковки задним ходом
#define GO_TO_PARKING                             5                     //  Позиция движения на парковку.
#define STOP_BEFORE_GO_TO_ANGLE                   6                     //  Позиция перед движением с парков к повороту на 90 градусов
#define GO_TO_ANGLE                               7                     //  Позиция движения к повороту на 90 градусов
#define GO_TO_SNAKE_FIRST                         8                     //  Позиция движения к первому  датчику змейки
#define GO_TO_SNAKE_SECOND                        9                     //  Позиция движения к второму  датчику змейки
#define GO_TO_SNAKE_THIRD                         10                    //  Позиция движения к третьему датчику змейки
#define GO_TO_FINISH                              11                    //  Позиция движения к финишу.
#define CROSS_FINISH_LINE                         12                    //  Позиция после пересечения финишной линии
#define GAME_OVER                                 13                    //  Позиция наезда на бордюр или окончания заезда
//----------------------------------------------------------------------//
//  УСТАНОВКИ МОДУЛЕЙ И ДАТЧИКОВ:
//----------------------------------------------------------------------//
#define NUMBER_OF_SENS_ON_EXPANDER                8                     //  Количество датчиков, подключенных к Trema Expander'у (расширитель выводов)
#define DECREASE_COEFFICIENT_FOR_SET_SENS_BOARD   0.07f                 //  Коэффициент задания нижней границы значений относительно верхней границы (при автокалибровке)
#define FIRST_SNAKE_SENS                          A0                    //  Вывод, к которому подключен первый аналоговый датчик линии, расположенный на змейке
#define SECOND_SNAKE_SENS                         A1                    //  Вывод, к которому подключен второй аналоговый датчик линии, расположенный на змейке
#define THIRD_SNAKE_SENS                          A2                    //  Вывод, к которому подключен третий аналоговый датчик линии, расположенный на змейке
#define EEPROM_ADRESS                             0                     //  Адрес ячейки EEPROM, куда будет записан последний рекорд
#define RESET                                     5                     //  Вывод, к которому подключена кнопка, отвечающая за сброс рекорда
#define WAITING_STOP_TIME                         2000                  //  Время ожидания до начала движения на трассе (в разных местах)
#define ESTAKADA_WAITING_TIME                     5000                  //  Время ожидания на эстакаде.
#define RESTART_GAME_TIME                         7000                  //  Время включения надписи СТАРТ после финиша (перезапуск игры)
#define NUMBER_OF_DECIMAL_PLACES                  1                     //  Количество знаков после запятой
//----------------------------------------------------------------------//
//  НАСТРОЙКИ ПЛЕЕРА:
//----------------------------------------------------------------------//
#define SOUND_BEFORE_START                        1                     //  Звук стадиона
#define SOUND_OF_START_THE_ENGINE                 2                     //  Звук запуска двигателя
#define SOUND_GO_PARKING_BY_BACK                  3                     //  Звук парковки задним ходом
#define SOUND_BEEP                                4                     //  Звук сирены
#define SOUND_NEW_RECORD                          5                     //  Звук нового рекорда времени
#define SOUND_ESTAKADA_WAITING                    6                     //  Звук ожидания на эстакаде
#define SOUND_SNAKE_LINE_MUSIC                    7                     //  Звук финишной прямой
#define SOUND_OF_WORKING_MOTOR                    8                     //  Звук работающего мотора
#define SOUND_LOSING_GAME                         9                     //  Звук проигрыша
//----------------------------------------------------------------------//
//  ПОДКЛЮЧЕНИЕ БИБЛИОТЕК И ОБЪЯВЛЕНИЕ ОБЪЕКТОВ:
//----------------------------------------------------------------------//
#include "Wire.h"                                                       //  Подключаем библиотеку для работы с аппаратной шиной I2C.
#include "EEPROM.h"                                                     //  Подключаем библиотеку для работы с энергонезависимой памятью
#include "iarduino_4LED.h"                                              //  Подключаем библиотеку iarduino_4LED
iarduino_4LED DISP_COUNT(13, 12);                                       //  Объявляем объект для работы с функциями библиотеки iarduino_4LED, с указанием выводов дисплея ( CLK , DIO )
iarduino_4LED DISP_RECORD(7, 6);                                        //  Объявляем объект для работы с функциями библиотеки iarduino_4LED, с указанием выводов дисплея ( CLK , DIO )
//----------------------------------------------------------------------//
#include "iarduino_I2C_Relay.h"                                         //  Подключаем библиотеку для работы с реле и силовыми ключами.
iarduino_I2C_Relay PWRKEY_A(0x0A);                                      //  Объявляем объект PWRKEY_A для работы с функциями и методами библиотеки iarduino_I2C_Relay, указывая адрес модуля на шине I2C.
iarduino_I2C_Relay PWRKEY_B(0x0B);                                      //  Объявляем объект PWRKEY_B для работы с функциями и методами библиотеки iarduino_I2C_Relay, указывая адрес модуля на шине I2C.
iarduino_I2C_Relay PWRKEY_C(0x0C);                                      //  Объявляем объект PWRKEY_C для работы с функциями и методами библиотеки iarduino_I2C_Relay, указывая адрес модуля на шине I2C.
iarduino_I2C_Relay PWRKEY_D(0x0D);                                      //  Объявляем объект PWRKEY_D для работы с функциями и методами библиотеки iarduino_I2C_Relay, указывая адрес модуля на шине I2C.
iarduino_I2C_Relay PWRKEY_E(0x0E);                                      //  Объявляем объект PWRKEY_E для работы с функциями и методами библиотеки iarduino_I2C_Relay, указывая адрес модуля на шине I2C.
//----------------------------------------------------------------------//
#include "iarduino_I2C_Expander.h"                                      //  Подключаем библиотеку для работы с расширителем выводов.
iarduino_I2C_Expander EXPAND_F(0x0F);                                   //  Объявляем объект EXPAND_F для работы с функциями и методами библиотеки iarduino_I2C_Expander, указывая адрес модуля на шине I2C.
//----------------------------------------------------------------------//
#include "SoftwareSerial.h"                                             //  Подключаем библиотеку для работы с программным последовательным портом
#include "DFRobotDFPlayerMini.h"                                        //  Подключаем библиотеку для работы с MP3-плеером
SoftwareSerial MY_SOFTWARE_SERIAL(8, 9);                                //  Объявляем объект MY_SOFTWARE_SERIAL (RX, TX) для работы с программным последовательным портом, указав выводы подключения модуля
DFRobotDFPlayerMini MP3_PLAYER;                                         //  Объявляем объект MP3_PLAYER для работы с MP3-плеером
//----------------------------------------------------------------------//
#include "Adafruit_Thermal.h"                                           //  Подключаем библиотеку для работы с термопринтером
Adafruit_Thermal PRINTER(&Serial);                                      //  Объявляем объект PRINTER библиотеки Adafruit_Thermal, указывая ссылку на класс Serial, или Serial1, или Serial2 ...
//----------------------------------------------------------------------//
//  СОЗДАНИЕ ПЕРЕМЕННЫХ:
//----------------------------------------------------------------------//
float    FullGameTime;                                                  //  Создаём переменную для хранения значения времени заезда
float    Highscore      = 999.9;                                        //  Создаём переменную для хранения рекордного времени заезда
bool     Mp3DemoFlag    = true;                                         //  Создаём флаг для включения MP3-плеера на работу в демо-режиме
bool     StartRaceFlag  = true;                                         //  Создаём флаг для начала отсчёта времени с начала гонки
bool     NewRecordFlag  = false;                                        //  Создаём флаг для печати текста о новом рекорде времени на принтере
uint8_t  DemoCount      = 0;                                            //  Создаём переменную-счётчик для смены режима работы светодиодной ленты в демо-режиме
uint8_t  ArrowState     = 0;                                            //  Создаём переменную-счётчик для смены режима работы группы стрелок на отдельных участках трассы
uint8_t  TrackPositionCode;                                             //  Создаём переменную для хранения кода позиции игрока на трассе.
//----------------------------------------------------------------------//
uint16_t StartSens, EstakadatSens, StartParkingSens;                    //  Создаём переменные для хранения аналоговых значений датчиков линии
uint16_t StopParkingSens, FinishSens, BoardSensOne;                     //  Создаём переменные для хранения аналоговых значений датчиков линии
uint16_t BoardSensTwo, BoardSensThree, SnakeSensOne;                    //  Создаём переменные для хранения аналоговых значений датчиков линии
uint16_t SnakeSensTwo, SnakeSensThree;                                  //  Создаём переменные для хранения аналоговых значений датчиков линии
//----------------------------------------------------------------------//
uint16_t LowValStartSens, LowValEstakadaSens, LowValStartParkingSens;   //  Создаём переменные для хранения нижней границы значений датчиков линии
uint16_t LowValStopParkingSens, LowValFinishSens, LowValBoardSensOne;   //  Создаём переменные для хранения нижней границы значений датчиков линии
uint16_t LowValBoardSensTwo, LowValBoardSensThree, LowValSnakeSensOne;  //  Создаём переменные для хранения нижней границы значений датчиков линии
uint16_t LowValSnakeSensTwo, LowValSnakeSensThree;                      //  Создаём переменные для хранения нижней границы значений датчиков линии
//----------------------------------------------------------------------//
uint32_t WaitingCount;                                                  //  Время ожидания до перехода к следующему действию
uint32_t StartGameTime;                                                 //  Время начала гонки (с момента пересечения линии СТАРТа)
//----------------------------------------------------------------------//
//----------------------------------------------------------------------//
void setup() {
  MY_SOFTWARE_SERIAL.begin(9600);                           //  Инициируем связь с программным последовательным портом на скорости 9600 бит/сек.
  Serial.begin(9600);                                       //  Инициируем связь с аппаратным последовательным портом на скорости 9600 бит/сек.
  pinMode(RESET, INPUT);                                    //  Конфигурируем вывод RESET на работу в качестве входа.
  if (digitalRead(RESET)) {                                 //  Если кнопка нажата при старте скетча, то
    EEPROM.put(EEPROM_ADRESS, Highscore);                   //  сбросить рекорд в значение по умолчанию
    delay(100);                                             //  Задержка в 100мс для окончания записи
  }
  DISP_COUNT.begin();                                       //  Инициируем работу с четырехразрядным индикатором для вывода времени заезда
  DISP_RECORD.begin();                                      //  Инициируем работу с четырехразрядным индикатором для вывода лучшего времени
  EEPROM.get(EEPROM_ADRESS, Highscore);                     //  Считываем значение рекорда из энергонезависимой памяти в переменную Highscore
  MP3_PLAYER.begin(MY_SOFTWARE_SERIAL);                     //  Инициируем работу с MP3-плеером
  MP3_PLAYER.volume(30);                                    //  Устанавливаем уровень громкости плеера (от 10 до 30)
  PRINTER.begin(255);                                       //  Инициируем работу с термопринтером, установив максимальное значение параметра предварительного нагрева
  PRINTER.setSize('L');                                     //  Устанавливаем крупный размер шрифта 'L' (Large)
  PRINTER.justify('C');                                     //  Устанавливаем выравнивание текста по центру 'C' (Center)
  PRINTER.setLineHeight(20);                                //  Устанавливаем межстрочный интервал в 2,0 мм.
  PRINTER.println(F("----------"));                         //  Печатаем тестовую линию отрыва чека
  PRINTER.feed(3);                                          //  Прокручиваем ленту на 3 строки
  PWRKEY_A.begin();                                         //  Инициируем работу с модулем реле
  PWRKEY_B.begin();                                         //  Инициируем работу с модулем реле
  PWRKEY_C.begin();                                         //  Инициируем работу с модулем реле
  PWRKEY_D.begin();                                         //  Инициируем работу с модулем реле
  PWRKEY_E.begin();                                         //  Инициируем работу с модулем реле
  EXPAND_F.begin();                                         //  Инициируем работу с расширителем выводов
  PWRKEY_A.digitalWrite(ALL_CHANNEL, LOW);                  //  Выключаем все каналы модуля реле
  PWRKEY_B.digitalWrite(ALL_CHANNEL, LOW);                  //  Выключаем все каналы модуля реле
  PWRKEY_C.digitalWrite(ALL_CHANNEL, LOW);                  //  Выключаем все каналы модуля реле
  PWRKEY_D.digitalWrite(ALL_CHANNEL, LOW);                  //  Выключаем все каналы модуля реле
  PWRKEY_E.digitalWrite(ALL_CHANNEL, LOW);                  //  Выключаем все каналы модуля реле
  pinMode(FIRST_SNAKE_SENS,  INPUT);                        //  Конфигурируем вывод FIRST_SNAKE_SENS на работу в качестве входа.
  pinMode(SECOND_SNAKE_SENS, INPUT);                        //  Конфигурируем вывод SECOND_SNAKE_SENS на работу в качестве входа.
  pinMode(THIRD_SNAKE_SENS,  INPUT);                        //  Конфигурируем вывод THIRD_SNAKE_SENS на работу в качестве входа.
  for (int i = 0; i < NUMBER_OF_SENS_ON_EXPANDER; i++) {    //  Проходим циклом по всем аналоговым датчикам, подключенным к Trema Expander'у и
    EXPAND_F.pinMode(i, INPUT, ANALOG);                     //  Конфигурируем все выводы на работу в качестве аналогового входа.
  }
  TrackPositionCode = GAME_OVER;                            //  Переходим к позиции GAME_OVER (это потушит все стрелки и приведёт к переходу в позицию READY_TO_START).
  delay(2000);                                              //  Устанавливаем задержку для того, чтобы АЦП Expander'а настроился
  //--------------------------------------------------------//
  //  Калибруем значения границ для датчиков линии:
  //--------------------------------------------------------//
  StartSens               = EXPAND_F.analogRead(6);         //  Датчик начала движения.
  EstakadatSens           = EXPAND_F.analogRead(7);         //  Датчик эстакады.
  StartParkingSens        = EXPAND_F.analogRead(0);         //  Датчик указывающий совершить парковку.
  StopParkingSens         = EXPAND_F.analogRead(4);         //  Датчик парковки.
  FinishSens              = EXPAND_F.analogRead(3);         //  Датчик завершения трассы.
  BoardSensOne            = EXPAND_F.analogRead(1);         //  Датчик бордюра.
  BoardSensTwo            = EXPAND_F.analogRead(2);         //  Датчик бордюра.
  BoardSensThree          = EXPAND_F.analogRead(5);         //  Датчик бордюра.
  SnakeSensOne            = analogRead(FIRST_SNAKE_SENS);   //  Первый датчик змейки.
  SnakeSensTwo            = analogRead(SECOND_SNAKE_SENS);  //  Второй датчик змейки.
  SnakeSensThree          = analogRead(THIRD_SNAKE_SENS);   //  Третий датчик змейки.
  //--------------------------------------------------------//
  //  Нижняя граница датчика начала движения:
  //--------------------------------------------------------//
  LowValStartSens         = StartSens - (StartSens * DECREASE_COEFFICIENT_FOR_SET_SENS_BOARD);
  //--------------------------------------------------------//
  //  Нижняя граница датчика эстакады:
  //--------------------------------------------------------//
  LowValEstakadaSens      = EstakadatSens - (EstakadatSens * DECREASE_COEFFICIENT_FOR_SET_SENS_BOARD);
  //--------------------------------------------------------//
  //  Нижняя граница датчика, указывающего совершить парковку:
  //--------------------------------------------------------//
  LowValStartParkingSens  = StartParkingSens - (StartParkingSens * DECREASE_COEFFICIENT_FOR_SET_SENS_BOARD);
  //--------------------------------------------------------//
  //  Нижняя граница датчика парковки:
  //--------------------------------------------------------//
  LowValStopParkingSens   = StopParkingSens - (StopParkingSens * DECREASE_COEFFICIENT_FOR_SET_SENS_BOARD);
  //--------------------------------------------------------//
  //  Нижняя граница датчика завершения трассы:
  //--------------------------------------------------------//
  LowValFinishSens        = FinishSens - (FinishSens * DECREASE_COEFFICIENT_FOR_SET_SENS_BOARD);
  //--------------------------------------------------------//
  //  Нижняя граница датчика бордюра:
  //--------------------------------------------------------//
  LowValBoardSensOne      = BoardSensOne - (BoardSensOne * DECREASE_COEFFICIENT_FOR_SET_SENS_BOARD);
  //--------------------------------------------------------//
  //  Нижняя граница датчика бордюра:
  //--------------------------------------------------------//
  LowValBoardSensTwo      = BoardSensTwo - (BoardSensTwo * DECREASE_COEFFICIENT_FOR_SET_SENS_BOARD);
  //--------------------------------------------------------//
  //  Нижняя граница датчика бордюра:
  //--------------------------------------------------------//
  LowValBoardSensThree    = BoardSensThree - (BoardSensThree * DECREASE_COEFFICIENT_FOR_SET_SENS_BOARD);
  //--------------------------------------------------------//
  //  Нижняя граница датчика 1 змейки:
  //--------------------------------------------------------//
  LowValSnakeSensOne      = SnakeSensOne - (SnakeSensOne * DECREASE_COEFFICIENT_FOR_SET_SENS_BOARD);
  //--------------------------------------------------------//
  //  Нижняя граница датчика 2 змейки:
  //--------------------------------------------------------//
  LowValSnakeSensTwo      = SnakeSensTwo - (SnakeSensTwo * DECREASE_COEFFICIENT_FOR_SET_SENS_BOARD);
  //--------------------------------------------------------//
  //  Нижняя граница датчика 3 змейки:
  //--------------------------------------------------------//
  LowValSnakeSensThree    = SnakeSensThree - (SnakeSensThree * DECREASE_COEFFICIENT_FOR_SET_SENS_BOARD);
}
//------------------------------------------------------------------//
//------------------------------------------------------------------//
void loop() {
  //----------------------------------------------------------------//
  //  Обновляем значения датчиков:
  //----------------------------------------------------------------//                                             CLOSE / OPEN
  StartSens        = EXPAND_F.analogRead(6);                        //  Датчик начала движения.                   ( 2400 / 4094 )
  EstakadatSens    = EXPAND_F.analogRead(7);                        //  Датчик эстакады.                          ( 1900 / 4094 )
  StartParkingSens = EXPAND_F.analogRead(0);                        //  Датчик указывающий совершить парковку.    ( 1200 / 2537 )
  StopParkingSens  = EXPAND_F.analogRead(4);                        //  Датчик парковки.                          ( 3300 / 4094 )
  FinishSens       = EXPAND_F.analogRead(3);                        //  Датчик завершения трассы.                 ( 2330 / 4094 )
  BoardSensOne     = EXPAND_F.analogRead(1);                        //  Датчик бордюра.                           ( 1650 / 3060 )
  BoardSensTwo     = EXPAND_F.analogRead(2);                        //  Датчик бордюра.                           ( 2500 / 4094 )
  BoardSensThree   = EXPAND_F.analogRead(5);                        //  Датчик бордюра.                           ( 3300 / 4094 )
  SnakeSensOne     = analogRead(FIRST_SNAKE_SENS);                  //  Первый датчик змейки.                     ( 500  / 1009 )
  SnakeSensTwo     = analogRead(SECOND_SNAKE_SENS);                 //  Второй датчик змейки.                     ( 400  / 1000 )
  SnakeSensThree   = analogRead(THIRD_SNAKE_SENS);                  //  Третий датчик змейки.                     ( 500  / 1000 )
  //----------------------------------------------------------------//
  //  Наезд на бордюр:
  //----------------------------------------------------------------//
  //  Если был совершён наезд на один из датчиков бордюра И установлен любой другой режим, кроме GAME_OVER, то
  if (TrackPositionCode != GAME_OVER && (BoardSensOne < LowValBoardSensOne || BoardSensTwo < LowValBoardSensTwo || BoardSensThree < LowValBoardSensThree)) {
    MP3_PLAYER.play(SOUND_LOSING_GAME);                             //  Включаем указанный трек
    TrackPositionCode = GAME_OVER;                                  //  Переходим к позиции GAME_OVER.
    Mp3DemoFlag = true;                                             //  Устанавливаем флаг включения предстартового трека
    StartRaceFlag = true;                                           //  Устанавливаем флаг готовности к новому заезду
    delay(3500);                                                    //  Задержка для того, чтобы мелодия проигрыша была проиграна до конца
  }
  //----------------------------------------------------------------//
  //  Подсчёт игрового времени:
  //----------------------------------------------------------------//
  if (!StartRaceFlag) {                                             //  Если флаг ожидания начала гонки сброшен, то
    FullGameTime = (float)(millis() - StartGameTime) / 1000;        //  вычисляем время прошедшее с момента пересечения датчика СТАРТ в секундах с плавающей точкой
    DISP_COUNT.print(FullGameTime, NUMBER_OF_DECIMAL_PLACES);       //  Выводим время прошедшее с момента пересечения датчика СТАРТ с 1 знаком после запятой.
  }
  //----------------------------------------------------------------//
  //  ПЕРЕКЛЮЧАЕМ ПОЗИЦИИ ДВИЖЕНИЯ:
  //----------------------------------------------------------------//
  // Позиция ожидания начала заезда:
  //----------------------------------------------------------------//
  if (TrackPositionCode == READY_TO_START) {
    PWRKEY_E.digitalWrite(1, HIGH);                                 //  Включаем надпись СТАРТ.
    if (StartSens < LowValStartSens) {                              //  Если зафиксирован наезд на датчик в линии старта, то ...
      PWRKEY_E.digitalWrite(1, LOW);                                //  Выключаем надпись СТАРТ.
      TrackPositionCode = START_RACE;                               //  Переходим к позиции START_RACE.
      MP3_PLAYER.play(SOUND_OF_START_THE_ENGINE);                   //  Включаем указанный трек
      StartGameTime = millis();                                     //  Фиксируем время пересечения датчика СТАРТ дл начала отсчёта времени
      StartRaceFlag = false;                                        //  Сбрасываем флаг ожидания начала гонки, чтобы начать считать время
    }
  }
  //----------------------------------------------------------------//
  //  Позиция после пересечения датчика СТАРТ:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == START_RACE) {
    switch (ArrowState) {                                           //  Переключаем стрелки направления движения, чтобы они включались змейкой, показывая направление движения
      case 0:
        PWRKEY_E.analogWrite(2, 4095);
        PWRKEY_E.analogWrite(3, 1024);
        PWRKEY_A.analogWrite(4, 1024);
        break;
      case 1:
        PWRKEY_E.analogWrite(2, 1024);
        PWRKEY_E.analogWrite(3, 4095);
        PWRKEY_A.analogWrite(4, 1024);
        break;
      case 2:
        PWRKEY_E.analogWrite(2, 1024);
        PWRKEY_E.analogWrite(3, 1024);
        PWRKEY_A.analogWrite(4, 4095);
        break;
    }
    ArrowState++; if (ArrowState > 2) ArrowState = 0;               //  Если счётчик переключения превысил количество переключаемых состояний, то сбрасываем его в 0
    if (EstakadatSens < LowValEstakadaSens) {                       //  Если зафиксирован наезд на датчик ЭСТАКАДЫ, то ...
      ArrowState = 0;                                               //  обнуляем счётчик включения элементов
      MP3_PLAYER.loop(SOUND_ESTAKADA_WAITING);                      //  Включаем указанный трек
      PWRKEY_E.digitalWrite(2, LOW); PWRKEY_E.digitalWrite(3, LOW); //  Отключаем первые стрелки после надписи старт.
      TrackPositionCode = ESTAKADA_WAITING;                         //  Переходим к позиции ESTAKADA_WAITING.
      WaitingCount = millis() + ESTAKADA_WAITING_TIME;              //  Определяем время съезда с эстакады.
    }
  }
  //----------------------------------------------------------------//
  //  Позиция нахождения на эстакаде:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == ESTAKADA_WAITING) {
    PWRKEY_A.digitalWrite(4, HIGH);                                 //  Включаем надпись СТОП на ЭСТАКАДЕ.
    if (EstakadatSens < LowValEstakadaSens) {                       //  Если машина находится на эстакаде, то ...
      if (millis() > WaitingCount) {                                //  если истекло время нахождения на ЭСТАКАДЕ, то ...
        MP3_PLAYER.play(SOUND_OF_WORKING_MOTOR);                    //  включаем указанный трек
        PWRKEY_A.digitalWrite(4, LOW);                              //  Отключаем надпись СТОП на ЭСТАКАДЕ.
        TrackPositionCode = GO_FROM_ESTAKADA;                       //  Переходим к позиции GO_FROM_ESTAKADA.
      }
    } else {                                                        //  Если машина съехала с эстакады РАНЬШЕ времени, то ...
      WaitingCount = millis() + ESTAKADA_WAITING_TIME;              //  Переопределяем новое время съезда с эстакады.
    }
  }
  //----------------------------------------------------------------//
  //  Позиция после эстакады:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == GO_FROM_ESTAKADA) {
    switch (ArrowState) {                                           //  Переключаем стрелки направления движения, чтобы они включались змейкой, показывая направление движения
      case 0:
        PWRKEY_A.analogWrite(2, 4095);
        PWRKEY_A.analogWrite(1, 1024);
        PWRKEY_B.analogWrite(3, 1024);
        break;
      case 1:
        PWRKEY_A.analogWrite(2, 1024);
        PWRKEY_A.analogWrite(1, 4095);
        PWRKEY_B.analogWrite(3, 1024);
        break;
      case 2:
        PWRKEY_A.analogWrite(2, 1024);
        PWRKEY_A.analogWrite(1, 1024);
        PWRKEY_B.analogWrite(3, 4095);
        break;
    }
    ArrowState++; if (ArrowState > 2) ArrowState = 0;               //  Если счётчик переключения превысил количество переключаемых состояний, то сбрасываем его в 0
    if (StartParkingSens < LowValStartParkingSens) {                //  Если зафиксирован наезд на датчик перед надписью СТОП в углу, то ...
      ArrowState = 0;                                               //  обнуляем счётчик включения элементов
      PWRKEY_A.digitalWrite(1, LOW); PWRKEY_A.digitalWrite(2, LOW); //  Отключаем первые стрелки после эстакады.
      TrackPositionCode = STOP_BEFORE_PARKING;                      //  Переходим к позиции STOP_BEFORE_PARKING.
      WaitingCount = millis() + WAITING_STOP_TIME;                  //  Определяем время работы блока в следующей позиции
    }
  }
  //----------------------------------------------------------------//
  //  Позиция ожидания начала движения на парковку:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == STOP_BEFORE_PARKING) {
    PWRKEY_B.digitalWrite(3, HIGH);                                 //  Включаем надпись СТОП в углу
    if (millis() > WaitingCount) {                                  //  Если время работы блока истекло, то...
      PWRKEY_B.digitalWrite(3, LOW);                                //  выключаем надпись СТОП в углу
      MP3_PLAYER.play(SOUND_GO_PARKING_BY_BACK);                    //  Включаем указанный трек
      TrackPositionCode = GO_TO_PARKING;                            //  Переходим к позиции GO_TO_PARKING.
    }
  }
  //----------------------------------------------------------------//
  //  Позиция движения на парковку:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == GO_TO_PARKING) {
    switch (ArrowState) {                                           //  Переключаем стрелки направления движения, чтобы они включались змейкой, показывая направление движения
      case 0:
        PWRKEY_A.analogWrite(3, 4095);
        PWRKEY_D.analogWrite(2, 1024);
        break;
      case 1:
        PWRKEY_A.analogWrite(3, 1024);
        PWRKEY_D.analogWrite(2, 4095);
        break;
    }
    ArrowState++; if (ArrowState > 1) ArrowState = 0;               //  Если счётчик переключения превысил количество переключаемых состояний, то сбрасываем его в 0
    if (StopParkingSens < LowValStopParkingSens) {                  //  Если зафиксирован наезд на датчик парковки, то ...
      ArrowState = 0;                                               //  обнуляем счётчик включения элементов
      MP3_PLAYER.play(SOUND_BEEP);                                  //  Включаем указанный трек
      PWRKEY_D.digitalWrite(2, LOW);                                //  Отключаем надпись стоп и стрелку на парковку.
      TrackPositionCode = STOP_BEFORE_GO_TO_ANGLE;                  //  Переходим к позиции STOP_BEFORE_GO_TO_ANGLE.
      WaitingCount = millis() + WAITING_STOP_TIME;                  //  Определяем время работы блока в следующей позиции
    }
  }
  //----------------------------------------------------------------//
  //  Позиция ожидания начала движения с парковки к датчику в углу:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == STOP_BEFORE_GO_TO_ANGLE) {
    PWRKEY_A.digitalWrite(3, HIGH);                                 //  Включаем надпись СТОП на ПАРКОВКЕ
    if (millis() > WaitingCount) {                                  //  Если время работы блока истекло, то...
      ArrowState = 0;                                               //  обнуляем счётчик включения элементов
      PWRKEY_A.digitalWrite(3, LOW);                                //  выключаем надпись СТОП на ПАРКОВКЕ
      MP3_PLAYER.play(SOUND_OF_WORKING_MOTOR);                      //  Включаем указанный трек
      TrackPositionCode = GO_TO_ANGLE;                              //  Переходим к позиции GO_TO_ANGLE.
    }
  }
  //----------------------------------------------------------------//
  //  Позиция движения к первому датчику в углу:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == GO_TO_ANGLE) {
    switch (ArrowState) {                                           //  Переключаем стрелку направления движения, чтобы она включалась змейкой, показывая направление движения
      case 0:
        PWRKEY_D.analogWrite(1, 4095);
        break;
      case 1:
        PWRKEY_D.analogWrite(1, 1024);
        break;
    }
    ArrowState++; if (ArrowState > 1) ArrowState = 0;               //  Если счётчик переключения превысил количество переключаемых состояний, то сбрасываем его в 0
    if (StartParkingSens < LowValStartParkingSens) {                //  Если зафиксирован наезд на датчик перед поротом в 90 градусов, то ...
      ArrowState = 0;                                               //  обнуляем счётчик включения элементов
      PWRKEY_D.digitalWrite(1, LOW);                                //  Отключаем стрелку в углу
      TrackPositionCode = GO_TO_SNAKE_FIRST;                        //  Переходим к позиции GO_TO_SNAKE_FIRST.
    }
  }
  //----------------------------------------------------------------//
  //  Позиция движения к первому датчику змейки:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == GO_TO_SNAKE_FIRST) {
    switch (ArrowState) {                                           //  Переключаем стрелки направления движения, чтобы они включались змейкой, показывая направление движения
      case 0:
        PWRKEY_B.analogWrite(1, 4095);
        PWRKEY_B.analogWrite(2, 1024);
        break;
      case 1:
        PWRKEY_B.analogWrite(1, 1024);
        PWRKEY_B.analogWrite(2, 4095);
        break;
    }
    ArrowState++; if (ArrowState > 1) ArrowState = 0;               //  Если счётчик переключения превысил количество переключаемых состояний, то сбрасываем его в 0
    if (SnakeSensOne < LowValSnakeSensOne) {                        //  Если зафиксирован наезд на первый датчик змейки, то ...
      ArrowState = 0;                                               //  обнуляем счётчик включения элементов
      PWRKEY_B.digitalWrite(1, LOW); PWRKEY_B.digitalWrite(2, LOW); //  Отключаем стрелки направления к ЗМЕЙКЕ
      MP3_PLAYER.play(SOUND_SNAKE_LINE_MUSIC);                      //  Включаем указанный трек
      TrackPositionCode = GO_TO_SNAKE_SECOND;                       //  Переходим к позиции GO_TO_SNAKE_SECOND.
    }
  }
  //----------------------------------------------------------------//
  //  Позиция движения к второму датчику змейки:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == GO_TO_SNAKE_SECOND) {
    switch (ArrowState) {                                           //  Переключаем стрелки направления движения, чтобы они включались змейкой, показывая направление движения
      case 0:
        PWRKEY_D.analogWrite(3, 4095);
        PWRKEY_E.analogWrite(4, 1024);
        break;
      case 1:
        PWRKEY_D.analogWrite(3, 1024);
        PWRKEY_E.analogWrite(4, 4095);
        break;
    }
    ArrowState++; if (ArrowState > 1) ArrowState = 0;               //  Если счётчик переключения превысил количество переключаемых состояний, то сбрасываем его в 0
    if (SnakeSensTwo < LowValSnakeSensTwo) {                        //  Если зафиксирован наезд на второй датчик змейки, то ...
      ArrowState = 0;                                               //  обнуляем счётчик включения элементов
      PWRKEY_D.digitalWrite(3, LOW); PWRKEY_E.digitalWrite(4, LOW); //  Отключаем стрелки
      TrackPositionCode = GO_TO_SNAKE_THIRD;                        //  Переходим к позиции GO_TO_SNAKE_THIRD.
    }
  }
  //----------------------------------------------------------------//
  //  Позиция движения к третьему датчику змейки:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == GO_TO_SNAKE_THIRD) {
    switch (ArrowState) {                                           //  Переключаем стрелки направления движения, чтобы они включались змейкой, показывая направление движения
      case 0:
        PWRKEY_D.analogWrite(4, 4095);
        PWRKEY_C.analogWrite(3, 1024);
        break;
      case 1:
        PWRKEY_D.analogWrite(4, 1024);
        PWRKEY_C.analogWrite(3, 4095);
        break;
    }
    ArrowState++; if (ArrowState > 1) ArrowState = 0;               //  Если счётчик переключения превысил количество переключаемых состояний, то сбрасываем его в 0
    if (SnakeSensThree < LowValSnakeSensThree) {                    //  Если зафиксирован наезд на третий датчик змейки, то ...
      ArrowState = 0;                                               //  обнуляем счётчик включения элементов
      PWRKEY_D.digitalWrite(4, LOW); PWRKEY_C.digitalWrite(3, LOW); //  Отключаем стрелки
      TrackPositionCode = GO_TO_FINISH;                             //  Переходим к позиции GO_TO_FINISH.
    }
  }
  //----------------------------------------------------------------//
  //  Позиция движения к финишу:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == GO_TO_FINISH) {
    switch (ArrowState) {                                           //  Переключаем стрелку и надпись ФИНИШ, чтобы они включались змейкой, показывая направление движения
      case 0:
        PWRKEY_C.analogWrite(2, 4095);
        PWRKEY_C.analogWrite(1, 1024);
        break;
      case 1:
        PWRKEY_C.analogWrite(2, 1024);
        PWRKEY_C.analogWrite(1, 4095);
        break;
    }
    ArrowState++; if (ArrowState > 1) ArrowState = 0;               //  Если счётчик переключения превысил количество переключаемых состояний, то сбрасываем его в 0
    if (FinishSens < LowValFinishSens) {                            //  Если зафиксирован наезд на ФИНИШ, то ...
      if (FullGameTime < Highscore) {                               //  если время заезда оказалось МЕНЬШЕ текущего рекорда времени, то...
        NewRecordFlag = true;                                       //  Устанавливаем флаг нового рекорда для дальнейшей печати термо-принтера
        MP3_PLAYER.play(SOUND_NEW_RECORD);                          //  Включаем указанный трек
        Highscore = FullGameTime;                                   //  Обновляем рекорд времени и
        EEPROM.put(EEPROM_ADRESS, Highscore);                       //  записываем его в энергонезависимую память
        DISP_RECORD.print(Highscore, NUMBER_OF_DECIMAL_PLACES);     //  Обновляем значение рекорда на индикаторе рекордов
      } else {                                                      //  Если же время заезда БОЛЬШЕ времени рекорда, то
        MP3_PLAYER.play(SOUND_BEEP);                                //  Включаем указанный трек
      }
      StartRaceFlag = true;                                         //  Устанавливаем флаг готовности трассы к новому заезду
      ArrowState = 0;                                               //  Обнуляем счётчик включения элементов
      PRINTER.setSize('L');                                         //  Устанавливаем крупный размер шрифта 'L' (Large)
      PRINTER.justify('C');                                         //  Устанавливаем выравнивание текста по центру 'C' (Center)
      PRINTER.println(F("iArduino Racing"));                        //  Выводим текст
      PRINTER.setLineHeight(60);                                    //  Устанавливаем межстрочный интервал в 6,0 мм.
      PRINTER.println(F("2020"));                                   //  Выводим текст
      PRINTER.println(F(""));                                       //  Выводим текст
      PRINTER.setLineHeight(80);                                    //  Устанавливаем межстрочный интервал в 8,0 мм.
      PRINTER.setSize('M');                                         //  Устанавливаем средний размер шрифта 'M' (Middle)
      PRINTER.println(F("Your time:"));                             //  Выводим текст
      PRINTER.setSize('L');                                         //  Устанавливаем крупный размер шрифта 'L' (Large)
      PRINTER.println(FullGameTime + (String)" seconds");           //  Выводим текст
      if (NewRecordFlag) {                                          //  Если установлен флаг нового рекорда скорости, то...
        NewRecordFlag = false;                                      //  Сбрасываем флаг нового рекорда
        PRINTER.setLineHeight(60);                                  //  Устанавливаем межстрочный интервал в 6,0 мм.
        PRINTER.println(F("A new record!"));                        //  Выводим текст о новом рекорде
      }
      PRINTER.setLineHeight(70);                                    //  Устанавливаем межстрочный интервал в 7,0 мм.
      PRINTER.setSize('M');                                         //  Устанавливаем средний размер шрифта 'M' (Middle)
      PRINTER.justify('L');                                         //  Устанавливаем выравнивание текста по левому краю 'L' (Left)
      PRINTER.println(F("Your name:"));                             //  Выводим текст
      PRINTER.println(F("Your  tel:"));                             //  Выводим текст
      PRINTER.feed(2);                                              //  Прокручиваем ленту на 2 строки
      PWRKEY_C.digitalWrite(2, LOW);                                //  Выключаем элементы перед финишной линией
      TrackPositionCode = CROSS_FINISH_LINE;                        //  Переходим к позиции CROSS_FINISH_LINE.
      WaitingCount = millis() + RESTART_GAME_TIME;                  //  Определяем время работы блока в следующей позиции
    }
  }
  //----------------------------------------------------------------//
  //  Позиция нахождения на финише:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == CROSS_FINISH_LINE) {
    PWRKEY_C.digitalWrite(1, HIGH);                                 //  Включаем надпись ФИНИШ
    if (millis() > WaitingCount) {                                  //  Если время работы блока истекло, то...
      PWRKEY_C.digitalWrite(1, LOW);                                //  выключаем надпись ФИНИШ
      TrackPositionCode = GAME_OVER;                                //  Переходим к позиции GAME_OVER
      Mp3DemoFlag = true;                                           //  Устанавливаем флаг включения предстартового трека
    }
  }
  //----------------------------------------------------------------//
  //  Позиция окончания игры или проигрыша:
  //----------------------------------------------------------------//
  else if (TrackPositionCode == GAME_OVER) {
    if (Mp3DemoFlag == true) {                                      //  Если флаг включения предстартового трека установлен, то
      Mp3DemoFlag = false;                                          //  сбрасываем флаг включения предстартового трека и
      MP3_PLAYER.loop(SOUND_BEFORE_START);                          //  включаем нужный трек в бесконечном повторе
    }
    DISP_COUNT.print( "----" );                                     //  Вывод текста ожидания начала гонки на индикатор
    DISP_RECORD.print(Highscore, NUMBER_OF_DECIMAL_PLACES);         //  Выводим на индикатор значение текущего рекорда времени
    DemoCount = 0;                                                  //  Обнуляем счётчик режима работы элементов в демо-режиме
    demo_resume();                                                  //  Включаем демонстрационный режим работы платформы

    //  Если зафиксирован наезд на датчик в линии ФИНИША и установлен режим GAME_OVER, то ...
    if (FinishSens < LowValFinishSens && TrackPositionCode == GAME_OVER) {
      PWRKEY_A.digitalWrite(ALL_CHANNEL, LOW);                      //  Выключаем все каналы модуля.
      PWRKEY_B.digitalWrite(ALL_CHANNEL, LOW);                      //  Выключаем все каналы модуля.
      PWRKEY_C.digitalWrite(ALL_CHANNEL, LOW);                      //  Выключаем все каналы модуля.
      PWRKEY_D.digitalWrite(ALL_CHANNEL, LOW);                      //  Выключаем все каналы модуля.
      PWRKEY_E.digitalWrite(ALL_CHANNEL, LOW);                      //  Выключаем все каналы модуля.
      TrackPositionCode = READY_TO_START;                           //  Переходим к позиции READY_TO_START
    }
  }
}
//------------------------------------------------------------------//
//------------------------------------//
// Демо-режим работы стенда
//------------------------------------//
void demo_resume() {
  DemoCount++;                        //  Увеличиваем счётчик для перехода к следующей позиции работы элемента платформы
  if (DemoCount > 18) DemoCount = 0;  //  Если значение счётчика превысило количество элементов на платформе - сбрасываем его в 0
  switch (DemoCount) {                //  Переключаем все элементы платформы, чтобы они включались змейкой по очереди
    case 0:
      PWRKEY_E.analogWrite(1, 4095); PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512);  PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512);  PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512);  PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512);  PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512);  PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 512); break;
    case 1:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 4095); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 512);  PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 512);  PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 512);  PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 512);  PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 512);  PWRKEY_C.analogWrite(1, 512); break;
    case 2:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 4095);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 512);  break;
    case 3:
      PWRKEY_E.analogWrite(1, 512);  PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 4095); PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512);  PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512);  PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512);  PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512);  PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 512); break;
    case 4:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 512);  PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 4095); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 512);  PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 512);  PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 512);  PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 512);  PWRKEY_C.analogWrite(1, 512); break;
    case 5:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 4095);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 512);  break;
    case 6:
      PWRKEY_E.analogWrite(1, 512);  PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512);  PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 4095); PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512);  PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512);  PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512);  PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 512); break;
    case 7:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 512);  PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 512);  PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 4095); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 512);  PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 512);  PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 512);  PWRKEY_C.analogWrite(1, 512); break;
    case 8:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 4095);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 512);  break;
    case 9:
      PWRKEY_E.analogWrite(1, 512);  PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512);  PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512);  PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 4095); PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512);  PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512);  PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 512); break;
    case 10:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 512);  PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 512);  PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 512);  PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 4095); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 512);  PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 512);  PWRKEY_C.analogWrite(1, 512); break;
    case 11:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 4095);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 512);  break;
    case 12:
      PWRKEY_E.analogWrite(1, 512);  PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512);  PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512);  PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512);  PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 4095); PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512);  PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 512); break;
    case 13:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 512);  PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 512);  PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 512);  PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 512);  PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 4095); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 512);  PWRKEY_C.analogWrite(1, 512); break;
    case 14:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 4095);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 512);  break;
    case 15:
      PWRKEY_E.analogWrite(1, 512);  PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512);  PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512);  PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512);  PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512);  PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 4095); PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 512); break;
    case 16:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 512);  PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 512);  PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 512);  PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 512);  PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 512);  PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 4095); PWRKEY_C.analogWrite(1, 512); break;
    case 17:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 4095); break;
    case 18:
      PWRKEY_E.analogWrite(1, 512); PWRKEY_E.analogWrite(2, 512); PWRKEY_E.analogWrite(3, 512);
      PWRKEY_A.analogWrite(4, 512); PWRKEY_A.analogWrite(2, 512); PWRKEY_A.analogWrite(1, 512);
      PWRKEY_B.analogWrite(3, 512); PWRKEY_D.analogWrite(2, 512); PWRKEY_A.analogWrite(3, 512);
      PWRKEY_D.analogWrite(1, 512); PWRKEY_B.analogWrite(2, 512); PWRKEY_B.analogWrite(1, 512);
      PWRKEY_D.analogWrite(3, 512); PWRKEY_E.analogWrite(4, 512); PWRKEY_D.analogWrite(4, 512);
      PWRKEY_C.analogWrite(3, 512); PWRKEY_C.analogWrite(2, 512); PWRKEY_C.analogWrite(1, 512); break;
  }
}
