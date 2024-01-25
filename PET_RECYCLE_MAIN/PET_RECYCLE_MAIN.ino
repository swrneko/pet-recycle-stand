#define PID_INTEGER //тип вычислений ПИД регулятора
#define PID_OPTIMIZED_I //оптимизация интегральной суммы ПИД регулятора
#define GS_FAST_PROFILE 32  //количество отрезков разгона/торможения
#include <LiquidCrystal_I2C.h>
#include <GyverNTC.h>
#include <GyverEncoder.h>
#include <GyverButton.h>
#include <PinChangeInterrupt.h>
#include <GyverStepper2.h>
#include <GyverTimers.h>
#include <GyverPID.h>
#include <PIDtuner.h>

//НАСТРОЙКИ=====================================================
#define max_temp 290 //максимальная температура
#define max_temp_tolerance 10 //допуск максимальной температуры. если не знаете, что это означает, то не трогайте. по умолчанию 10.
#define temp_correction -3  //различие температуры с датчика и калибровки термопарой. если температура по факту больше, то без знака, если меньше то с минусом. по умолчанию 0.
#define sp_max 9 //максимальная скорость
#define LED_STOP_PIN 14 //пин светодиода остановки (А0)
#define LED_HEAT_PIN 15 //пин светодиода кнопки нагрева (А1)
#define LED_SPIN_PIN 16 //пин светодиода кнопки вращени (А2)
#define ENC_S1 3  //синий
#define ENC_S2 2  //коричневый 
#define ENC_KEY 4 //зелёный
#define BTN_PIN1 7  //кнопка включения/выключения нагревателя
#define BTN_PIN2 6  //кнопка запуска/остановки вращения
#define endstop_pin 13 //пин концевика
#define STEP_PIN 8 //пин step (коричневый)
#define DIR_PIN 11 //пин direction (зелёный)
#define EN_PIN 12 //пин enable (бело-зелёный)
#define STEPS 3200  //количество шагов двигателя (200) * разрешение шага (1/16)
#define ACCSEL 500  //установка ускорения в шаг/сек^2
#define KP 45 //пропорциональный коэффициент
#define KI 3  //коэффициент интегрирующей составляющей
#define KD 200  //коэффициент дифференциальной составляющей
#define DT 500  //время итерации регулятора в мс
#define PWM_HEATING_PIN 5  //пин транзистора шим регулятора нагревателя
#define MIN_LIM_PID_OUT 0 //минимальное ограничение выходного сигнала с пид регулятора (для ШИМ регулировки нагрева необходимо оставить равной "0")
#define MAX_LIM_PID_OUT 255 //максимальное ограничение выходного сигнала с пид регулятора (для ШИМ регулировки нагрева необходимо оставить равной "255")
#define PID_DIR NORMAL //направление регулирования калибровки коэффициентов ПИД регулятора (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
#define SIGNAL 100 //сигнал калибровки коэффициентов ПИД регулятора
#define DELTA_SIGNAL 50  //конечный сигнал калибровки коэффициентов ПИД регулятора
#define PERIOD 2000 //период калибровки коэффициентов ПИД регулятора
#define IMPULSE 10000
#define ACCURACY 1  //точность калибровки коэффициентов ПИД регулятора
#define PERIOD_OF_ITERARION 500 //период итерации калибровки коэффициентов ПИД регулятора
//============================================================

//ИНИЦИАЛИЗАЦИЯ==============================================
GButton butt1(BTN_PIN1);  //инициализация кнопки 1
GButton butt2(BTN_PIN2);  //инициализация кнопки 2
Encoder enc(2, 3, 4, 1); //инициализация энкодера S2 S1 KEY (clk, dt, sw)
GyverNTC therm(3, 1150, 4000);  //инициализация термистора
LiquidCrystal_I2C lcd(0x27, 16, 2); //инициализация дисплея | SDA-A4(бело-зелёный) SCL-A5(бело-коричневый)
GStepper2<STEPPER2WIRE> stepper(STEPS, STEP_PIN, DIR_PIN, EN_PIN);  //объявление мотора nema 17
GyverPID regulator(KP, KI, KD, DT); //инициализация ПИД регулятора
PIDtuner tuner;  //инициализация автотюнера коэффициентов ПИД регулятора
//============================================================

//СПЕЦ. СИМВОЛЫ=========================================================
byte heat[] = {0x03, 0x0B, 0x08, 0x1C, 0x08, 0x08, 0x09, 0x06}; //значёк нагрева
byte spin[] = {0x00, 0x10, 0x1E, 0x1D, 0x01, 0x11, 0x0E, 0x00};  //значёк вращения
//============================================================

//ПЕРЕМЕННЫЕ============================================================
unsigned long tmr = 0;  //таймер
unsigned long tmr_plotter = 0;  //таймер плоттера
unsigned long tmr_animation_heating_1 = 0; //таймер анимации нагрева 1
unsigned long tmr_animation_heating_2 = 0; //таймер анимации нагрева 2
unsigned long tmr_pid = 0;  //таймер ПИД регулятора
int set_temp = 0; //установка температуры
int sp = 0; //установка скорости
int arrow = 0;  //положение стрелки
int temp_old = 0; //старая температура
int set_temp_old = 0; //старая установка температуры
int sp_old = 0;  //старая установка скорости
bool heat_flag = false; //состояние нагрева
bool spining_flag = false; //состояние вращения
bool clear_stop_flag = false; //состояние отчистки при остановке концевиком
//======================================================================

void setup() {
  //==========================================================================================
  Serial.begin(9600); //подключение сериал порта
  Serial.flush(); //дождаться остановки подачи информации в последовательный порт для чистоты графиков плоттера
  Serial.println("temp,set_temp,output"); //название графиков
  stepper.disable();  //отключение шагового двигателя
  //==========================================================================================
  pinMode(LED_STOP_PIN, OUTPUT);  //настройка пина светодиода индикации остановки всех процессов
  pinMode(LED_HEAT_PIN, OUTPUT);  //настройка пина светодиода индикации нагрева
  pinMode(LED_SPIN_PIN, OUTPUT);  //настройка пина светодиода индикации вращения
  pinMode(EN_PIN, OUTPUT);  //найстройка пина вкл/выкл драйвера шагового двигателя как выход
  pinMode(PWM_HEATING_PIN, OUTPUT); //настройка пина транзистора с ШИМ выводом как выход
  pinMode(endstop_pin, INPUT_PULLUP); //подключение концевика
  stepper.setAcceleration(ACCSEL);  //установка ускорения в шаг/сек^2
  stepper.autoPower(true);  //автоупровление вкл/выкл шагового двигателя
  regulator.setLimits(MIN_LIM_PID_OUT, MAX_LIM_PID_OUT);  //установка ограничений выходного сигнала ПИД регулятора
  regulator.setDirection(NORMAL); // направление регулирования (NORMAL/REVERSE). ПО УМОЛЧАНИЮ СТОИТ NORMAL
  tuner.setParameters(PID_DIR, SIGNAL, DELTA_SIGNAL, PERIOD, ACCURACY, IMPULSE, PERIOD_OF_ITERARION); //параметры тюнера коэффициентов ПИД регулятора
  lcd.init(); //инициализация
  lcd.backlight();  //включить подсветку
  Timer1.enableISR(); //подключаем прерывания для мотора
  Timer1.setPeriod(20); //период таймера прерывания (для мотора) в мкс
  lcd.createChar(0, heat);  //загружаем значёк нагрева
  lcd.createChar(1, spin);  //загружаем значёк вращения
  attachInterrupt(0, isr, CHANGE);    // прерывание на 2 пине! CLK у энка
  attachPCINT(BTN_PIN1, bnt1_isr, CHANGE);  //прерывание кнопки 1
  attachPCINT(BTN_PIN2, bnt2_isr, CHANGE);  //прерывание кнопки 2
  attachPCINT(endstop_pin, endstop_isr, CHANGE);  //прерывание концевика
  //==========================================================================================
  lcd.home(); //сброс курсора
  lcd.print("PET plastic"); //Вывод надписи
  lcd.setCursor(5, 1);      // PET plastic
  lcd.print("recycling");   //         recycling
  delay(2000);
  lcd.clear(); //отчистка дисплея
  lcd.print("By Alexander");  //Вывод надписи
  lcd.setCursor(8, 1);        // By Alexander
  lcd.print("Egorov");        //        Egorov
  delay(2000);
  lcd.clear(); //отчистка дисплея
  lcd.home();
  lcd.print("temp:");
  lcd.setCursor(10, 0);
  lcd.print("sp:");
  lcd.print("R");
  lcd.setCursor(0, 1);
  lcd.print("set_temp:");
  lcd.print(set_temp);
  lcd.print("C");
  lcd.write(223);
  //==========================================================================================
}

void loop() {
  enc.tick(); //опрос энкодер
  butt1.tick(); //опрос кнопки 1
  butt2.tick(); //опрос кнопки 2
  timer_period_setup(); //изменение периода таймера прерывания для шагового двигателя
  // plotter(); //вывод графика в serial port *НЕОБХОДИМО ЗАКОМЕНТИРОВАТЬ ПОСЛЕ КАЛИБРОВКИ!!!!!*
  // tuner_pid_k();  //калибровка коэффициентов ПИД регулятора *НЕОБХОДИМО ЗАКОМЕНТИРОВАТЬ ПОСЛЕ КАЛИБРОВКИ!!!!!*
  temp_gui(); //вывод температуры в меню
  set_temp_gui(); //вывод устаовки температуры в меню
  sp_gui(); //вывод скорости вращения в меню
  stepper_spining();  //изменение скорости вращения мотора
  arrow_gui();  //вывод указателя в меню
  arrow_change(); //смена параметров в меню
  heating (); //включение/выключение нагрева
  heating_point();  //отображение нагрева
  PID_algorithm();  //алгоритм регулирования ПИД  *НЕОБХОДИМО ЗАКОМЕНТИРОВАТЬ НА ВРЕМЯ КАЛИБРОВКИ КАЛИБРОВКИ!!!!!*
  spining();  //включение/выключение вращения
  spining_point();  //отображение вращения
  stop_all_process_endstop();  //остановка всех процессов по окончанию ленты
}

void plotter() {  //вывод графика в serial port
  if (millis() - tmr_plotter >= 1000) {
    Serial.print(filtered_temp());
    Serial.print(",");
    Serial.print(set_temp);
    Serial.print(",");
    Serial.println(regulator.output);
    tmr_plotter = millis();
  }
}

void arrow_gui() {  //вывод указателя в меню
  if (enc.isRight()) arrow +=1; //обработка поворота вправо энкодера
  if (arrow > 1) {
    arrow = 1;
  }
  if (enc.isLeft()) arrow -=1;  //обработка поворота вправо энкодера
  if (arrow < 0) {
    arrow = 0;
  }
  if (arrow == 0) { //стирание предыдущей стрелки и отрисовка новой
    lcd.print(" ");
    lcd.setCursor(14, 0);
    lcd.write(127);
    lcd.setCursor(14, 1);
    lcd.print(" ");
  }
  if (arrow == 1) { //стирание предыдущей стрелки и отрисовка новой
    lcd.print(" ");
    lcd.setCursor(14, 0);
    lcd.print(" ");
    lcd.setCursor(14, 1);
    lcd.write(127);
  }
}

void temp_gui() { //вывод температуры в меню
  if(temp_old != filtered_temp() && millis() - tmr >= 1000) { //проверка изменения температуры
      lcd.setCursor(5, 0);
      lcd.print("     ");
      lcd.setCursor(5, 0);
      lcd.print(filtered_temp());
      lcd.print("C");
      lcd.write(223);
      temp_old = filtered_temp();
      tmr = millis();
  }
}

void set_temp_gui() { //вывод устаовки температуры в меню
  if (set_temp_old != set_temp) { //проверка изменения установки температуры
    lcd.setCursor(9, 1);    //стирание старой установки температуры и вывод новой
    lcd.print("     ");
    lcd.setCursor(9, 1);
    lcd.print(set_temp);
    lcd.print("C");
    lcd.write(223);
    set_temp_old = set_temp;
  }
}

void sp_gui() { //вывод скорости вращения в меню
  if (sp_old != sp) {  //проверка изменения скорости вращения
    if (sp == 0) {
      lcd.setCursor(13, 0); //стирание старой и вывод новой скорости вращения
      lcd.print("  ");
      lcd.setCursor(13, 0);
      lcd.print("R");
      sp_old = sp;
    }
    if (sp != 0) {
      lcd.setCursor(13, 0); //стирание старой и вывод новой скорости вращения
      lcd.print("  ");
      lcd.setCursor(13, 0);
      lcd.print(sp);
      sp_old = sp;
    }
  }
}

void PID_algorithm() {  //ПИД регулятор
  if (heat_flag == true) {
    regulator.input = round(filtered_temp()); //ввод температурного значения с термистора в регулятор
    regulator.setpoint = set_temp; //установки температуры
    pid_heating(); //нагрев с функцией отключения нагрева при превышении максимальной температуры + допуск
  }
  if (heat_flag == false) {
    analogWrite(PWM_HEATING_PIN, 0);
  }
}

void tuner_pid_k() {  //авторюнер коэффициентов ПИД регулятора
  //Коротко о процессе регулировки пид регулятора тюнером.
  //1. Надо раскомментировать функцию tuner_pid_k() в цикле loop();
  //2. Открыть монитор порта и дождаться окончания калибровки (в окне монитора порта будут отображены коэффициенты ПИД);
  //Эти коэффициенты нужно будет ввести вместо "0" к переменным ПИД коэфиициентов в блоке "НАСТРОЙКИ";
  //3. Обратно закомментировать данную функцию.
  //Готово! ПИД коэффициенты удачно откалиброванны!
  tuner.setInput(round(therm.getTempAverage()));  //передаём текущее значение температуры
  tuner.compute();  //вычисления тюнера
  analogWrite(PWM_HEATING_PIN, tuner.getOutput());  //подача ШИМ сигнала на нагревательный блок
  tuner.debugText();  //вывод информации о тюнере коэффициентов ПИД регулятора
}

void heating() { //включение/выключение нагрева
  if (heat_flag == false) {
    if (butt1.isClick()) {
      digitalWrite(LED_HEAT_PIN, true); //включение светодиода нагрева
      heat_flag = true;
    }
  }
  if (heat_flag == true) {
    if (butt1.isClick()) {
      heat_flag = false;
      digitalWrite(LED_HEAT_PIN, false); //отключение светодиода нагрева
      lcd.setCursor(15, 1); //отчистка значка нагревателя
      lcd.print(" ");
    }
  }
}

void spining() {  //включение/выключение вращения
  if (spining_flag == false) {
    if (butt2.isClick()) {
      spining_flag = true;
      digitalWrite(LED_SPIN_PIN, true); //включение светодиода вращения
      stepper.enable(); //включение вращения мотора
    }
  }
  if (spining_flag == true) {
    if (butt2.isClick()) {
      spining_flag = false;
      digitalWrite(LED_SPIN_PIN, false); //отключение светодиода вращения
      stepper.stop();
      stepper.disable();
      lcd.setCursor(15, 0); //отчистка значка вращения
      lcd.print(" ");
    }
  }
}

void arrow_change() { //смена параметров в меню
  if ((arrow == 0) && (enc.isRightH())) {
    sp += 1;
    max_min_sp(); //ограничение максимальной и минимальной скорости
  }
  if ((arrow == 0) && (enc.isLeftH())) {
    sp -= 1;
    max_min_sp(); //ограничение максимальной и минимальной скорости
  }
  if ((arrow == 1) && (enc.isRightH())) {
    set_temp +=5;
    min_max_temp(); //ограничение максимальной и минимальной температурой
  }
  if ((arrow == 1) && (enc.isLeftH())) {
    set_temp -= 5;
    min_max_temp(); //ограничение максимальной и минимальной температурой
  }
}

void stop_all_process_endstop() { //остановка всех процессов по окончанию ленты
  if (digitalRead(endstop_pin) == true) { //проверяем состояние концевика на присутствие ленты, если нет леты, то 
    if (clear_stop_flag == false) { //если значки не были удалены, то удаляем
      lcd.setCursor(15, 1); //отчистка значка нагревателя
      lcd.print(" ");
      lcd.setCursor(15, 0); //отчистка значка вращения
      lcd.print(" ");
      clear_stop_flag = true;
    }
    spining_flag = false; //отключение вращения
    heat_flag = false;  //отключение нагрева
    digitalWrite(LED_STOP_PIN, true); //включение светодиода остановки
    lcd.setCursor(15, 0);
    lcd.print("X");
    lcd.setCursor(15, 1);
    lcd.print("X");
  }
  if (digitalRead(endstop_pin) == false) {  //проверяем состояние концевика на присутствие ленты, если есть лента, то
    if (clear_stop_flag == true) {  //если значки остановки были отрисованны, то удаляем 
      lcd.setCursor(15, 0); //отчистка значка вращения
      lcd.print(" ");
      lcd.setCursor(15, 1); //отчистка значка нагревателя
      lcd.print(" ");
      digitalWrite(LED_STOP_PIN, false); //отключение светодиода остановки
      clear_stop_flag = false;
    }
  }
}

void stepper_spining() {  //вращение шагового двигателя
  if (spining_flag == true) {  //если шаговый двигатель вращается, то выбор скорости
    if (sp == 1) {
      stepper.reverse(false);
      stepper.setSpeed(400);
    }
    if (sp == 2) {
      stepper.reverse(false);
      stepper.setSpeed(800);
    }
    if (sp == 3) {
      stepper.reverse(false);
      stepper.setSpeed(1800);
    }
    if (sp == 4) {
      stepper.reverse(false);
      stepper.setSpeed(2200);
    }
    if (sp == 5) {
      stepper.reverse(false);
      stepper.setSpeed(2800);
    }
    if (sp == 6) {
      stepper.reverse(false);
      stepper.setSpeed(3200);
    }
    if (sp == 7) {
      stepper.setSpeed(3600);
    }
    if (sp == 8) {
      stepper.reverse(false);
      stepper.setSpeed(4800);
    }
    if (sp == 9) {
      stepper.reverse(false);
      stepper.setSpeed(5600);
    }
    if (sp == 0) {  //если установка скорости равна "0", то включение реверсивного вращения
      stepper.reverse(true);
      stepper.setSpeed(4800);
    }
  }
}

int filtered_temp() { //функция фильтрации температуры
  return filtered_temp_algorithm(round(therm.getTempAverage())) + temp_correction;
}

int filtered_temp_algorithm(float newVal) { //алгоритм фильтрации температуры
  float k = 0.8;  // коэффициент фильтрации, 0.0-1.0
  static int filVal = 0;
  filVal += (newVal - filVal) * k;
  return filVal;
}

void pid_heating() { //нагрев с функцией отключения нагрева при превышении максимальной температуры + допуск
  if (filtered_temp() <= max_temp) {
    analogWrite(PWM_HEATING_PIN, regulator.output); //подача ШИМ сигнала на нагрвевательный блок
  }
  if (filtered_temp() > max_temp + max_temp_tolerance) {
    analogWrite(PWM_HEATING_PIN, 0);
  }
}

void heating_point() {  //отображение нагрева
  if (heat_flag == true) {
    lcd.setCursor(15, 1);
    lcd.write(0);
  }
}

void spining_point() {  //отображение вращения
  if (spining_flag == true) {
    lcd.setCursor(15, 0);
    lcd.write(1);
  }
}

void max_min_sp() { //ограничение максимальной и минимальной скорости
  if (sp > sp_max) {
    sp = sp_max;
  }
  if (sp < 0) sp = 0;
}

void min_max_temp() { //ограничение максимальной и минимальной температурой
  if (set_temp > max_temp)  set_temp = max_temp;
  if (set_temp < 0) set_temp = 0;
}

void timer_period_setup() { //изменение периода таймера прерывания для шагового двигателя
  Timer1.setPeriod(stepper.getPeriod());
}

ISR(TIMER1_A) { //обработка шагового двигателя и ПИД регулятора в прерывании по таймеру
  stepper.tickManual();
  if (millis() - tmr_pid >= DT) {
    regulator.getResult();
    tmr_pid = millis();
  }
}

void isr() {  //отработка энкодера в прерывании
  enc.tick();  
}

void bnt1_isr() { //отработка кнопки 1 в прерывании
  butt1.tick();
}

void bnt2_isr() { //отработка кнопки 2 в прерывании
  butt2.tick();
}

void endstop_isr() {  //обработка концевика в прерывании
  if (digitalRead(endstop_pin == true));
}