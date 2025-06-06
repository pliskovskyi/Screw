#include <pwm.h>
#include <PID_v1_bc.h>

// Инициализация LCD дисплея
  #include <Wire.h>
  #include <hd44780.h>
  #include <hd44780ioClass/hd44780_I2Cexp.h>
  hd44780_I2Cexp lcd;

// Инициализация энкодера
  #include "GyverEncoder.h"
  #define CLK 7
  #define DT 5
  #define SW 6
  Encoder enc1(CLK, DT, SW);

//Инициализаци двигателей
  #define dutyCycle 24.41  //наполниение сигнала в процентах
  #define freq_circle 1600  //импульсов на один оборот
  #define freq_max 8000  //максимальное число импульсов - соответсвует 300 оборотам в минуту или 5 оборотов в секунду
  #define freq_step 13.33 //шаг изменения, соответствет 3 градуса или 0,5 оборота в минуту 

  double freq = freq_step;  //переменная через которую задается частота на двигатель
  uint32_t period_us = 1000000 / freq;  //пересчет на длительность импульса в наносекундах для подачи команды на процессор
  double rpm = 60 * freq / freq_circle;  // пересчет в обороты в минуту
  #define time_step 500  // период задержки для изменения частоты
  uint32_t time_check = 0;
  #define time_endstop 2000 // миллисекунд которые не считывается значение энкодера при его срабатывании - время для размыкания контакта
  uint32_t time_check_endstop = 0;

  bool change_flow = 1;  //разрешение на использование поворота ручки энкодеранаправление движения каретки
  bool dir_car_check = 1; //направление движения каретки
  bool pid_control = false;
  int pid_control_new = 0;
  int pid_control_old = 0;
  bool PIDautoTune = true;
  #define EndStop1 A2
  #define EndStop2 A1
  uint8_t EndStop_count = 0;

  #define en 4
  #define dir 2
  PwmOut pwm3(3);

  #define en_car 8
  #define dir_car 10
  PwmOut pwm9(9);

//Инициализация Балерины 
  #define Lazer A0
  #define LazerTests 5000  //количество считываний для среднего изменения балерины на экране при ручном управлении - раз в пол секунды-раз в секунду
  uint32_t LazerCheck = 0; //накопительная переменная где суммируются все считывания датчика на протяжении измерений в количестве LazerTests
  uint32_t LazerCheckNow = 0; //Среднее значение крайнего расчета
  uint32_t LazerCount = 0; //накопление количества тестов для сравнения с LazerTests


  #define Lazer_Tests_PIDcal 2500 //количество считывания для автоуправления - много раз в секунду
  int Lazer_Setpoint = 250 ;//Позиция Балерины - чуть ниже чем 45 градусов - 350, почти на нуде 100
  uint32_t Lazer_Check_PIDcal = 0; //накопительная переменная где суммируются все считывания датчика на протяжении измерений в количестве LazerTestsPIDcal
  uint32_t Lazer_CheckNow_PIDcal = 0; //Среднее значение крайнего расчета
  uint32_t Lazer_CheckOld_1_PIDcal = 0; //Среднее значение крайнего расчета
  uint32_t Lazer_CheckOld_2_PIDcal = 0; //Среднее значение крайнего расчета
  int32_t Lazer_Speed_Now = 0; //Изенение между Lazer_CheckNow_PIDcal и Lazer_CheckOld_1_PIDcal
  int32_t Lazer_Speed_Old = 0; //Изенение между Lazer_CheckOld_1_PIDcal и Lazer_CheckOld_2_PIDcal
  uint32_t Lazer_Count_PIDcal = 0; //накопление количества тестов для сравнения с LazerTestsPIDcal
  #define interval 500 // Интервал в 500 миллисекунд для изменения шага при авто-скорости, аналог time_step
  unsigned long previousMillis = 0;  //контроль interval срабатывания изменения скорости
  unsigned long previousMillisLaser = 0;  //контроль interval срабатывания изменения скорости
  #define to_thick 10  //минимальное значение Балерины, хотя фактически рычаг не опускается ниже 57
  #define to_thin 900 //максимальное значение Балерины, хотя фактически рычан не поднимается выше 860
 
//Инициализация кнопки
  #define button1 11

// ПИД-регулятор
  uint8_t Math_multi = 10;
  double Setpoint = 300, Input, Output;
  double Kp = 0.60, Ki = 0.00, Kd = 18.71;
  PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Параметры для калибровки
  bool calibrating = false;
  double Kp_critical;
  double Tu;
  bool oscillating = false;
  unsigned long lastTime;
  unsigned long periodStart;
  int oscillationCount = 0;
  const int oscillationTarget = 10;




void setup() {
  pinMode(en_car, OUTPUT);
  digitalWrite(en_car, LOW);
  pinMode(dir_car, OUTPUT);
  digitalWrite(dir_car, HIGH);
  pinMode(dir, OUTPUT);
  digitalWrite(dir, HIGH);
  pinMode(EndStop1, INPUT_PULLUP);
  pinMode(EndStop2, INPUT_PULLUP);
  pinMode(button1, INPUT_PULLUP);

  enc1.setType(TYPE2);
  Serial.begin(115200);

  while (!Serial) {
    ; // Ожидаем инициализации сериал порта
  }
  Serial.println("Serial initialized");

  lcd.begin(20, 4);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("   HAND             ");
  lcd.setCursor(0, 1);
  lcd.print(" --------           ");
  lcd.setCursor(0, 2);
  lcd.print("freq  rpm ");
  lcd.setCursor(0, 3);
  lcd.print(freq);
  lcd.setCursor(6, 3);
  lcd.print(rpm);


  Serial.println("LCD initialized");

  pwm3.begin();
  pwm3.period_us(period_us);
  pwm3.pulse_perc(dutyCycle);
  pwm9.begin();
  pwm9.period_us(period_us * 2);
  pwm9.pulse_perc(dutyCycle);

  Serial.println("PWM initialized");

  myPID.SetMode(AUTOMATIC);
  pid_control_new = digitalRead(button1);
  pid_control_old = pid_control_new;
}
void loop() {
  pid_control_new = digitalRead(button1);
  if (pid_control_new == 0) {
//    Serial.println(" вкл ");
    pid_control = true;
    if (pid_control_new != pid_control_old) {
      Serial.println("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx-----CHANGE-------xxxxxxxxxxxxxxxxxxxxxxxxxxxx");
      //обнуляем данные авто-скорости при любом переключении. 
      Lazer_Check_PIDcal = 0; //накопительная переменная где суммируются все считывания датчика на протяжении измерений в количестве LazerTestsPIDcal
      Lazer_CheckNow_PIDcal = 0; //Среднее значение крайнего расчета
      Lazer_CheckOld_1_PIDcal = 0; //Среднее значение крайнего расчета
      Lazer_CheckOld_2_PIDcal = 0; //Среднее значение крайнего расчета
      Lazer_Speed_Now = 0; //Изенение между Lazer_CheckNow_PIDcal и Lazer_CheckOld_1_PIDcal
      Lazer_Speed_Old = 0; //Изенение между Lazer_CheckOld_1_PIDcal и Lazer_CheckOld_2_PIDcal
      Lazer_Count_PIDcal = 0;
//      delay(1000);
    }
  } else {
//    Serial.println(" выкл ");
    pid_control = false;
    if (pid_control_new != pid_control_old) {
      Serial.println("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx-----CHANGE-------xxxxxxxxxxxxxxxxxxxxxxxxxxxx");
      //обнуляем данные авто-скорости при любом переключении. 
      Lazer_Check_PIDcal = 0; //накопительная переменная где суммируются все считывания датчика на протяжении измерений в количестве LazerTestsPIDcal
      Lazer_CheckNow_PIDcal = 0; //Среднее значение крайнего расчета
      Lazer_CheckOld_1_PIDcal = 0; //Среднее значение крайнего расчета
      Lazer_CheckOld_2_PIDcal = 0; //Среднее значение крайнего расчета
      Lazer_Speed_Now = 0; //Изенение между Lazer_CheckNow_PIDcal и Lazer_CheckOld_1_PIDcal
      Lazer_Speed_Old = 0; //Изенение между Lazer_CheckOld_1_PIDcal и Lazer_CheckOld_2_PIDcal
      Lazer_Count_PIDcal = 0;
  //    delay(1000);
    }
  }
  pid_control_old = pid_control_new;

  enc1.tick();

 if (pid_control) {
    pidControlOperation();
  } else {
    normalOperation();
  }
}
void normalOperation() {
  if ((millis() - time_check_endstop) > time_endstop) {
    if ((analogRead(EndStop1) < 200) || (analogRead(EndStop2) < 200)) {
      if (EndStop_count < 3) {
        EndStop_count = EndStop_count + 1;
      } else {
        EndStop_count = 0;
        dir_car_check = !dir_car_check;
        digitalWrite(dir_car, dir_car_check ? HIGH : LOW);
        time_check_endstop = millis();
        Serial.print("---------------------CHANGE-----------------------");
        Serial.println(dir_car_check);
  //     delay(1000);
      }
    } else {
      EndStop_count = 0;
    }
  }
   /* Serial.print(analogRead(EndStop1));
    Serial.print("/");
    Serial.println(analogRead(EndStop2));
*/
  LazerCheckFunction();


  if (enc1.isRight()) {
    if (change_flow) {
      if ((freq != freq_max) && (freq < freq_max)) {
        freq += freq_step;
        updatePWM();
        refreshNumbers();
        Serial.print(" rpm ");
        Serial.println(rpm);
 //       Serial.println(digitalRead(button1));
      }
    } 
  }

  if (enc1.isLeft()) {
    if (change_flow) {
      if ((freq != 0) && (freq >= freq_step)) {
        freq = max(freq - freq_step, 0);

        updatePWM();
        refreshNumbers();
        Serial.print(" rpm ");
        Serial.println(rpm);
//        Serial.println(digitalRead(button1));
      }
    } 
  }
}
void pidControlOperation() {
  // Контроль датчиков изменения направления перед управлением
  if ((millis() - time_check_endstop) > time_endstop) {
    if ((analogRead(EndStop1) < 200) || (analogRead(EndStop2) < 200)) {
      if (EndStop_count < 3) {
        EndStop_count = EndStop_count + 1;
      } else {
        EndStop_count = 0;
        dir_car_check = !dir_car_check;
        digitalWrite(dir_car, dir_car_check ? HIGH : LOW);
        time_check_endstop = millis();
        Serial.print("---------------------CHANGE-----------------------");
        Serial.println(dir_car_check);
      }
    } else {
      EndStop_count = 0;
    }
  }
  /*  Serial.print(analogRead(EndStop1));
    Serial.print("/");
    Serial.println(analogRead(EndStop2));
*/
  // Обновление значения Lazer
  LazerCheckFunctionPIDcal();

  if (enc1.isRight()) {
    Lazer_Setpoint = Lazer_Setpoint + 10;
    if (Lazer_Setpoint > 350) {Lazer_Setpoint = 350;}
    Serial.println("--------------------Setpoint------------------- ");
    Serial.print("                       ");
    Serial.println(Lazer_Setpoint);

  }

  if (enc1.isLeft()) {
    Lazer_Setpoint = Lazer_Setpoint - 10;
    if (Lazer_Setpoint < 70) {Lazer_Setpoint = 70;}
    Serial.println("--------------------Setpoint------------------- ");
    Serial.print("                       ");
    Serial.println(Lazer_Setpoint);
 
  }


  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    float Ves;
    float old_freq = freq;
    float maxSpeedChange = freq_step * 3.0; // Максимальное изменение скорости за один шаг
    
    if (Lazer_CheckNow_PIDcal > Lazer_Setpoint) {
      Ves = (float)((int32_t)Lazer_CheckNow_PIDcal - (int32_t)Lazer_Setpoint) / (float)Lazer_Setpoint;
      if (Lazer_Speed_Old > 0) { //старая скорость показывала движение Балерины от равновестия в натяжку
        if (Lazer_Speed_Now == Lazer_Speed_Old) {//движение от равновесия в ратяжку равномерное
                  freq = constrain(freq - freq_step*Math_multi*Ves, freq_step, freq_max);
        } else if (Lazer_Speed_Now > 0) { //однонаправленное движение без разворотов
          if (Lazer_Speed_Now < Lazer_Speed_Old) { // отклонение от равновесия с уменьшающейся скоростью, ожидается разворот
                  freq = constrain(freq - freq_step*Math_multi*0.5*Ves, freq_step, freq_max);
          } else if (Lazer_Speed_Now > Lazer_Speed_Old) {  // отклонение от равновесия с наростанием скорости
                  freq = constrain(freq - freq_step*Math_multi*2.0*Ves, freq_step, freq_max); 
          } 

        } else if (Lazer_Speed_Now < 0) { //фиксируются разовороты
          if (abs(Lazer_Speed_Now) < Lazer_Speed_Old) { // разворот к равновесию с гашением скорости - возможен повторный разоворот от равновестия или хз что
                  freq = constrain(freq - freq_step*Math_multi*0.1*Ves, freq_step, freq_max); 
          } else if (abs(Lazer_Speed_Now) > Lazer_Speed_Old) {  // разворот к равновесию с наростанием скорости
                  freq = constrain(freq + freq_step*Math_multi*0.1*Ves, freq_step, freq_max); 
          } 
        }

      } else if (Lazer_Speed_Old < 0) {//старая скорость показывала движение Балерины к равновесию
        if (Lazer_Speed_Now == Lazer_Speed_Old) {//движение к равновесию равномерное
                  freq = constrain(freq + freq_step*Math_multi*0.1*Ves, freq_step, freq_max);  
        } else if (Lazer_Speed_Now < 0) { //однонаправленное движение без разворотов
        if (abs(Lazer_Speed_Now) < abs(Lazer_Speed_Old)) { // возврат к равновесию с уменьшающейся скоростью, ожидается разворот
                   freq = constrain(freq - freq_step*Math_multi*0.1*Ves, freq_step, freq_max); 
          } else if (abs(Lazer_Speed_Now) > abs(Lazer_Speed_Old)) {  // возврат к равновесию с наростанием скорости
                   freq = constrain(freq + freq_step*Math_multi*0.5*Ves, freq_step, freq_max); 
          } 


        } else if (Lazer_Speed_Now > 0) { //фиксируются разовороты
          if (Lazer_Speed_Now < abs(Lazer_Speed_Old)) { // разворот от равновесия с гашением скорости - возможен повторный разоворот от равновестия или хз что
                   freq = constrain(freq - freq_step*Math_multi*0.5*Ves, freq_step, freq_max); 
          } else if (Lazer_Speed_Now > abs(Lazer_Speed_Old)) {  // разворот от равновесия с наростанием скорости
                   freq = constrain(freq + freq_step*Math_multi*Ves, freq_step, freq_max);  
          } 

        }
      }  else if (Lazer_Speed_Old == 0) {
                      freq = constrain(freq - freq_step*Math_multi*1.0*Ves, freq_step, freq_max); 
      }
    } else if (Lazer_CheckNow_PIDcal < Lazer_Setpoint) {
      Ves = Lazer_Setpoint - Lazer_CheckNow_PIDcal;
      Ves = Ves / Lazer_Setpoint;
      if (Lazer_Speed_Old > 0) { //старая скорость показывала движение Балерины снизу к равновестию
        if (Lazer_Speed_Now == Lazer_Speed_Old) {//движение к равновесию от ослабления равномерное
                    freq = constrain(freq - freq_step*Math_multi*0.1*Ves, freq_step, freq_max);  
        } else if (Lazer_Speed_Now > 0) { //однонаправленное движение без разворотов
          if (Lazer_Speed_Now < Lazer_Speed_Old) { // движение к равновесию с уменьшающейся скоростью, ожидается разворот
                    freq = constrain(freq + freq_step*Math_multi*0.1*Ves, freq_step, freq_max); 
          } else if (Lazer_Speed_Now > Lazer_Speed_Old) {  // движение к равновесию с наростанием скорости
                    freq = constrain(freq + freq_step*Math_multi*0.5*Ves, freq_step, freq_max); 
          } 

        } else if (Lazer_Speed_Now < 0) { //фиксируются разовороты
          if (abs(Lazer_Speed_Now) < Lazer_Speed_Old) { // разворот к ослаблению с гашением скорости - возможен повторный разоворот от равновестия или хз что
                    freq = constrain(freq + freq_step*Math_multi*0.1*Ves, freq_step, freq_max); 
          } else if (abs(Lazer_Speed_Now) > Lazer_Speed_Old) {  // разворот к ослаблению с наростанием скорости
                    freq = constrain(freq + freq_step*Math_multi*0.5*Ves, freq_step, freq_max); 
          } 


        }

      } else if (Lazer_Speed_Old < 0) {//старая скорость показывала движение Балерины к ослаблению
        if (Lazer_Speed_Now == Lazer_Speed_Old) {//движение от равновесия в ослабление равномерное
                      freq = constrain(freq + freq_step*Math_multi*Ves, freq_step, freq_max);
        } else if (Lazer_Speed_Now < 0) { //однонаправленное движение без разворотов
        if (abs(Lazer_Speed_Now) < abs(Lazer_Speed_Old)) { // движение в ослабление с уменьшающейся скоростью, ожидается разворот
                      freq = constrain(freq + freq_step*Math_multi*0.5*Ves, freq_step, freq_max);
          } else if (abs(Lazer_Speed_Now) > abs(Lazer_Speed_Old)) {  // движение в ослабление с наростанием скорости
                      freq = constrain(freq + freq_step*Math_multi*2.0*Ves, freq_step, freq_max); 
          } 


        } else if (Lazer_Speed_Now > 0) { //фиксируются разовороты
          if (Lazer_Speed_Now < abs(Lazer_Speed_Old)) { // разворот к равновесию с гашением скорости - возможен повторный разоворот от равновестия или хз что
                      freq = constrain(freq + freq_step*Math_multi*0.5*Ves, freq_step, freq_max); 
          } else if (Lazer_Speed_Now > abs(Lazer_Speed_Old)) {  // разворот к равновесию с наростанием скорости
                      freq = constrain(freq - freq_step*Math_multi*0.1*Ves, freq_step, freq_max);
          } 
        }
      } else if (Lazer_Speed_Old == 0) {
                      freq = constrain(freq + freq_step*Math_multi*1.0*Ves, freq_step, freq_max); 
      }
    }

    // Ограничиваем максимальное изменение скорости
    if (abs(freq - old_freq) > maxSpeedChange) {
      if (freq > old_freq) {
        freq = old_freq + maxSpeedChange;
      } else {
        freq = old_freq - maxSpeedChange;
      }
    }
    
    // Выводим диагностическую информацию только если частота изменилась
    if (freq != old_freq) {
      Serial.println("\n=== SPEED CHANGE ===");
      Serial.print("Time: "); Serial.print(millis()/1000); Serial.println("s");
      
      // Состояние натяжения
      Serial.print("Tension: "); Serial.print(Lazer_CheckNow_PIDcal);
      Serial.print(" (Setpoint: "); Serial.print(Lazer_Setpoint);
      int32_t tensionError = (int32_t)Lazer_CheckNow_PIDcal - (int32_t)Lazer_Setpoint; // Исправляем расчет ошибки
      Serial.print(", Error: "); Serial.print(tensionError);
      Serial.println(")");
      
      // Скорость изменения натяжения
      Serial.print("Tension Speed: "); Serial.print(Lazer_Speed_Now);
      Serial.print(" (Previous: "); Serial.print(Lazer_Speed_Old);
      Serial.println(")");
      
      // Изменение скорости двигателя
      Serial.print("Motor: "); Serial.print(old_freq); Serial.print("Hz -> ");
      Serial.print(freq); Serial.print("Hz (");
      Serial.print(rpm); Serial.print(" RPM)");
      float speedChange = freq - old_freq;
      Serial.print(" Change: "); Serial.print(speedChange);
      Serial.print("Hz (");
      Serial.print(speedChange * 60.0 / freq_circle);
      Serial.println(" RPM)");
      
      // Параметры регулирования
      float relativeError = (float)tensionError / (float)Lazer_Setpoint; // Исправляем расчет относительной ошибки
      Serial.print("Control: Ves="); Serial.print(relativeError);
      Serial.print(", Math_multi="); Serial.print(Math_multi);
      Serial.print(", freq_step="); Serial.print(freq_step);
      Serial.println("\n===================");
    }

    updatePWM();
    refreshNumbers();
  }
}


void LazerCheckFunctionPIDcal() {
  Lazer_Count_PIDcal++;  // Увеличиваем счетчик количества измерений
  Lazer_Check_PIDcal += analogRead(Lazer);  // Считываем данные с лазерного датчика и суммируем

 // if (Lazer_Count_PIDcal == Lazer_Tests_PIDcal) {  // Если достигли необходимого количества измерений
  unsigned long currentMillisLazer = millis();
  if (currentMillisLazer - previousMillisLaser >= (interval*0.49)) {
    previousMillisLaser = currentMillisLazer;
   
    if (Lazer_CheckOld_2_PIDcal > 0) { //когда уже было хотя-бы одно считывание данных
      Lazer_CheckOld_2_PIDcal = Lazer_CheckOld_1_PIDcal; //Пересохраняем историю
      Lazer_CheckOld_1_PIDcal = Lazer_CheckNow_PIDcal; //Сохраняем текущее значение в истории перед перезаписью
      //Lazer_CheckNow_PIDcal = Lazer_Check_PIDcal / Lazer_Tests_PIDcal;
      Lazer_CheckNow_PIDcal = Lazer_Check_PIDcal / Lazer_Count_PIDcal;
      if (Lazer_CheckNow_PIDcal >= 1000) Lazer_CheckNow_PIDcal = 999;  // Ограничиваем значение, если оно превышает 999
    } else { //если считываний не было и заполнение идет первый раз
      //Lazer_CheckNow_PIDcal = Lazer_Check_PIDcal / Lazer_Tests_PIDcal;
      Lazer_CheckNow_PIDcal = Lazer_Check_PIDcal / Lazer_Count_PIDcal;
      if (Lazer_CheckNow_PIDcal >= 1000) Lazer_CheckNow_PIDcal = 999;  // Ограничиваем значение, если оно превышает 999
      Lazer_CheckOld_1_PIDcal = Lazer_CheckNow_PIDcal; //дублируем все три поакзателя
      Lazer_CheckOld_2_PIDcal = Lazer_CheckOld_1_PIDcal;
    } 

    //пересчет скоростей 
    Lazer_Speed_Now = Lazer_CheckNow_PIDcal - Lazer_CheckOld_1_PIDcal; 
    Lazer_Speed_Old = Lazer_CheckOld_1_PIDcal - Lazer_CheckOld_2_PIDcal; 

    lcd.setCursor(17, 3);  // Обновляем отображение на дисплее
    lcd.print(Lazer_CheckNow_PIDcal);

    if (Lazer_CheckNow_PIDcal < 100) {
      lcd.setCursor(19, 3);
      lcd.print(" ");
    }

    // Обнуляем счетчики
    Lazer_Check_PIDcal = 0;
    Lazer_Count_PIDcal = 0;
  }
}


void LazerCheckFunction() {
  LazerCount++;
  LazerCheck += analogRead(Lazer);

  if (LazerCount == LazerTests) {
    LazerCheckNow = LazerCheck / LazerTests;
    if (LazerCheckNow >= 1000) LazerCheckNow = 999;

    lcd.setCursor(17, 3);
    lcd.print(LazerCheckNow);
   //  Serial.println(LazerCheckNow);

    if (LazerCheckNow < 100) {
      lcd.setCursor(19, 3);
      lcd.print(" ");
    }

    // Обнуление счетчиков
    LazerCheck = 0;
    LazerCount = 0;
  }
}

void refreshNumbers() {
  lcd.setCursor(0, 3);
  lcd.print(freq);
  if (freq < 1000) {
    lcd.setCursor(2, 3);
    lcd.print("  ");
  }
  rpm = 60 * freq / freq_circle;
  lcd.setCursor(6, 3);
  lcd.print(rpm);
  
  
  if (rpm < 100) {
    lcd.setCursor(8, 3);
    lcd.print("  ");
  }
}

void updatePWM() {
  period_us = 1000000 / freq;
  pwm3.period_us(period_us);
  pwm3.pulse_perc(dutyCycle);
  pwm9.period_us(period_us * 2);
  pwm9.pulse_perc(dutyCycle);
}

