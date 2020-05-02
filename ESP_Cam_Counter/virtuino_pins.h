
#ifndef VirtuinoH
#define VirtuinoH


#define V_memory_count 64          // the size of V memory. You can change it to a number <=255)
float V[V_memory_count];           // This array is synchronized with Virtuino V memory. You can change the type to int, long etc.
//---
boolean debug = false;              // set this variable to false on the finale code to decrease the request time.



//Опеределенее ячеек памяти на сервере virtuino

#define V_lastCommTime            0 //V0 время соединения используется для первоначального сброса видимости кнопок в приложении

#define V_number_of_sum_frames    1 //V1 - число кадров суммирования
#define V_offset_y                2 //V2 - смещенее по оси Y при суммировании кадров и отображении на дисплее 75 85 87
#define V_offset_x                3 //V3 - смещенее по оси X при суммировании кадров и отображении на дисплее
#define V_level_dispalay_ttf_B_W  4 //V4 - Доп. уровень бинаризации для дисплея 15
#define V_level_find_digital_Y    5 //V5 - Доп. уровень бинаризации для поиска цифр по y
#define V_level_find_digital_X    6 //V6 - Доп. уровень бинаризации для поиска цифр по X

#define V_level_convert_to_32     7 //V7 - Доп. уровень бинаризации при конвертации в 32 бита

#define V_level_Y_up              8 //V8 - Положенее шторки сверху Y_up
#define V_level_Y_down            9 //V9 - Положенее шторки снизу Y_down

#define V_show_digital           10 //V10 - номер цифры какую выводим на экран для сравнения 8 - нет вывода
#define V_offset_x_digital       11 //V11 - смещенее по оси X при отображении на дисплеи для анализа 50 100 150
#define V_GBW                    12 //V12 - 0 = gray 1 - b/w
#define V_SH_0_1                 13 //V13 - вывод на монитор значения гистограммы в двоичном виде
#define V_SH_HEX                 14 //V14 - вывод на монитор в HEX цифр шкалы
#define V_SH_M3                  15 //V15 - вывод на монитор результатов накопленного газа
#define V_m3                     16 //V16 - текущее значенее объема газа умноженное на 100
#define V_m3_minutes             17 //V17 - значенее минут при текущем объеме газа

#define V_D0                     18 //V18 - опознанная цифра 1
#define V_D1                     19 //V19 - опознанная цифра 2
#define V_D2                     20 //V20 - опознанная цифра 3
#define V_D3                     21 //V21 - опознанная цифра 4
#define V_D4                     22 //V22 - опознанная цифра 5
#define V_D5                     23 //V23 - опознанная цифра 6
#define V_D6                     24 //V24 - опознанная цифра 7
#define V_D7                     25 //V25 - опознанная цифра 8

#define V_error_recognition      26 //V26 - ошибки распознавания по сравнению с предыдущим
#define V_offset_y_test          27 //V27 - дополнительное смещение по оси Y
#define V_Sum_min_Hemming        28 //V28 - суммарное значенее всех расстояний Хемминга
#define V_RESTART                29 //V29  software restart
#define V_m3_m                   30 //V30 - текущее значенее объема газа за 1 минуту
#define V_offset_y_current       31 //V31 - текущее дополнительное смещение по оси Y
#define V_Sum_min_Hemming_current 32 //V32 - суммарное значенее всех расстояний Хемминга
#define V_Sum_min_Hemming_error  33 //V33 - ошибки большое суумарное значение расстояния Хемминга

#define V_CropX1  				 41 //V41 - Initial frame crop coordinates
#define V_CropX2  				 42 //V42 - Initial frame crop coordinates
#define V_CropY1  				 43 //V43 - Initial frame crop coordinates
#define V_CropY2  				 44 //V44 - Initial frame crop coordinates


String T_0 = "";                    // результаты распознавания
String T_1 = "";                    // расстояние Хемминга
String T_2 = "";                    // частоты повторения цифр

//предопределенные значения
#define old_number_of_sum_frames 5 //число кадров суммирования

#define old_offset_y 83 //смещенее по оси Y при суммировании кадров и отображении на дисплее 54  86
#define old_offset_x  0 //смещенее по оси X при суммировании кадров и отображении на дисплее

#define old_level_dispalay_ttf_B_W 30 //Доп. уровень бинаризации для дисплея 30
#define old_level_find_digital_Y 30 //Доп. уровень бинаризации для поиска цифр по y 30
#define old_level_find_digital_X 70 //Доп. уровень бинаризации для поиска цифр по X 60 70
#define old_level_convert_to_32 30 //Доп. уровень бинаризации при конвертации в 32 бита 15 73 25 50
#define old_level_Y_up 22 //Положенее шторки сверху Y_up 23
#define old_level_Y_down 48 //Положенее шторки снизу X_down 48


#include <EEPROM.h>

//адреса хранения предопределеных значений в EEPROM
#define offset_y_addr 0 //смещенее по оси Y при суммировании кадров и отображении на дисплее
#define offset_x_addr offset_y_addr + sizeof(byte) //смещенее по оси X при суммировании кадров и отображении на дисплее
#define level_find_digital_Y_addr offset_x_addr + sizeof(byte) //Доп. уровень бинаризации для поиска цифр по y
#define level_find_digital_X_addr level_find_digital_Y_addr + sizeof(byte) //Доп. уровень бинаризации для поиска цифр по X
#define level_convert_to_32_addr level_find_digital_X_addr + sizeof(byte) //Доп. уровень бинаризации при конвертации в 32 бита
#define level_Y_up_addr level_convert_to_32_addr + sizeof(byte)  //Положенее шторки сверху Y_up
#define level_Y_down_addr level_Y_up_addr + sizeof(byte) //Положенее шторки снизу X_down


//================================================================= check_limits
bool check_limits(float &V_test, uint8_t V_max, uint8_t V_min, uint8_t V_set, uint8_t V_MI, uint8_t V_addr)
{
  //Проверка на нахождение в допустимых прелах переменной V_test
  //V_test - значение, который нужно проверить
  //V_max - максимальное значение передела
  //V_min - минимальное значение передал
  //V_set - установить если вне предлов
  //V_MI - ячейка памяти в virtuino
  //V_addr - адрес ячейки для сохранения

//  printf("На входе: V_set %d\tV_test %f4\tV_old %d\n", V_set, V_test, EEPROM.readByte(V_addr));

  if ((uint8_t)(V_test) >= V_max || (uint8_t)(V_test) < V_min || isnan(V_test))
  { //если значение вне пределов или неопределено - nan
    //    Serial.print("Установка первоначального значения: V_set " + (String)V_set
    //                 + "\tV_test " + (String)V_test + "\tV_MI " + (String)V_MI + "\tV_V_addr " + (String)V_addr);
//    Serial.printf("Установка первоначального значения: V_set %f\tV_test %f\tV_MI %d\tV_addr %d\n", V_set, V_test, V_MI, V_addr);

    V_test = V_set; //не будет изменения восстанавливаем первоначальное

    V[V_MI] = V_test;

    if (EEPROM.readByte(V_addr) == (uint8_t)(V_test)) //если значение в памяти совпадает не записывать
      return false;

    EEPROM.writeByte(V_addr, (uint8_t)(V_test));     //сохранить в памяти EEPROM
    return true;
  }
  else
  {
    if (EEPROM.readByte(V_addr) != (uint8_t)(V_test)) //если значение в памяти не совпадает записать
    {
      EEPROM.writeByte(V_addr, (uint8_t)(V_test));
      printf("Установить и сохранить значение: V_set %d\tV_test %d\tV_MI %d\tV_addr %d\n", V_set, (uint8_t)(V_test), V_MI, V_addr);
      return true;
    }
    else
      return false;
  }
}
//================================================================= check_limits

//================================================================= change_variables
void change_variables(bool read_from_memory)
//read_from_memory true восстановить из памяти
{
  boolean write_EEPROM_flag = false;
  
  //смещенее по оси Y при суммировании кадров и отображении на дисплее 20 250
  if (read_from_memory) V[V_offset_y] = EEPROM.readByte(offset_y_addr); //восстановить значение из памяти
  write_EEPROM_flag |= check_limits(V[V_offset_y], 250, 20, old_offset_y, V_offset_y, offset_y_addr);

  //смещенее по оси X при суммировании кадров и отображении на дисплее 0 - 50
  if (read_from_memory) V[V_offset_x] = EEPROM.readByte(offset_x_addr); //восстановить значение из памяти
  write_EEPROM_flag |= check_limits(V[V_offset_x], 50, 0, old_offset_x, V_offset_x, offset_x_addr);
  
  //Доп. уровень бинаризации для поиска цифр по y 30
  if (read_from_memory) V[V_level_find_digital_Y] = EEPROM.readByte(level_find_digital_Y_addr); //восстановить значение из памяти
  write_EEPROM_flag |= check_limits(V[V_level_find_digital_Y], 100, 0, old_level_find_digital_Y, V_level_find_digital_Y, level_find_digital_Y_addr);
    
 //Доп. уровень бинаризации для поиска цифр по X 60 70 80
  if (read_from_memory) V[V_level_find_digital_X] = EEPROM.readByte(level_find_digital_X_addr); //восстановить значение из памяти
  write_EEPROM_flag |= check_limits(V[V_level_find_digital_X], 150, 0, old_level_find_digital_X, V_level_find_digital_X, level_find_digital_X_addr);

 //Доп. уровень бинаризации при конвертации в 32 бита 15 73 25 50
  if (read_from_memory) V[V_level_convert_to_32] = EEPROM.readByte(level_convert_to_32_addr); //восстановить значение из памяти
  write_EEPROM_flag |= check_limits(V[V_level_convert_to_32], 150, 0, old_level_convert_to_32, V_level_convert_to_32, level_convert_to_32_addr);
  
 //Положенее шторки сверху Y_up 23
  if (read_from_memory) V[V_level_Y_up] = EEPROM.readByte(level_Y_up_addr); //восстановить значение из памяти
  write_EEPROM_flag |= check_limits(V[V_level_Y_up], 100, 10, old_level_Y_up, V_level_Y_up, level_Y_up_addr);

 //Положенее шторки снизу X_down 44
  if (read_from_memory) V[V_level_Y_down] = EEPROM.readByte(level_Y_down_addr); //восстановить значение из памяти
  write_EEPROM_flag |= check_limits(V[V_level_Y_down], 150, 10, old_level_Y_down, V_level_Y_down, level_Y_down_addr);

  //запись данных в память
  if (write_EEPROM_flag)
  { //записать данные в память
    // EEPROM.commit(); //подтвердить запись в память // for debug period
    printf("Обновлены данные в памяти\n");
  }
}
//================================================================= change_variables

//================================================================= init_V
void init_V() {
  boolean write_EEPROM_flag = false;
  
  //инициализация переменных
  change_variables(true);

  V[V_number_of_sum_frames] = old_number_of_sum_frames; //число кадров суммирования
//  V[V_level_dispalay_ttf_B_W] = old_level_dispalay_ttf_B_W; //Доп. уровень бинаризации для дисплея 15
  V[V_level_dispalay_ttf_B_W] = V[V_level_convert_to_32]; //Доп. уровень бинаризации для дисплея совпдает с уровнем уровень бинаризации при конвертации в 32 бита
    

  V[V_show_digital] = 8;  //V10 - номер цифры какую выводим на экран для сравнения 8 - нет вывода
  V[V_offset_x_digital] = 0; //V11 - смещенее по оси X при отображении на дисплеи для анализа 50 100 150
  V[V_GBW] =  0; //V12 - 0-  - b/w 1 - gray 2 - gray full
  V[V_SH_0_1] = 0; //V13 - вывод на монитор значения гистограммы в двоичном виде
  V[V_SH_HEX] = 0; //V14 - вывод на монитор в HEX цифр шкалы
  V[V_SH_M3] = 0; //V15 - вывод на монитор результатов накопленного газа

  V[V_error_recognition] = 0.0; //ошибки распознавания нет

  V[V_offset_y_test] = 0.0; //дополнительное смещение по оси Y
  V[V_offset_y_current] = 0.0; //дополнительное смещение по оси Y
  V[V_Sum_min_Hemming_current] = 0;
  V[V_Sum_min_Hemming] = 0;
  V[V_Sum_min_Hemming_error] = 0;

  
  V[V_RESTART] = 0; //сбросить клавишу рестарта

  V[V_m3_m] = 0;
}
//================================================================= init_V

//================================================================= onReceived
/* This function is called every time Virtuino app sends a request to server to change a Pin value
   The 'variableType' can be a character like V, T, O  V=Virtual pin  T=Text Pin    O=PWM Pin
   The 'variableIndex' is the pin number index of Virtuino app
   The 'valueAsText' is the value that has sent from the app   */
void onReceived(char variableType, uint8_t variableIndex, String valueAsText) {
  if (variableType == 'V') {
    float value = valueAsText.toFloat();        // convert the value to float. The valueAsText have to be numerical
    if (variableIndex < V_memory_count) V[variableIndex] = value;          // copy the received value to arduino V memory array
  }
  /*
       else  if (variableType=='T'){
          if (variableIndex==0) T_0=valueAsText;          // Store the text to the text variable T0
          else if (variableIndex==1) T_1=valueAsText;     // Store the text to the text variable T1
         else if (variableIndex==2) T_2=valueAsText;     // Store the text to the text variable T2
         //else if (variableIndex==3) T_3=valueAsText;     // Store the text to the text variable T3

        }
  */
}
//================================================================= onReceived


//================================================================= onRequested
/* This function is called every time Virtuino app requests to read a pin value*/
String onRequested(char variableType, uint8_t variableIndex) {

  char S[30];

  if (variableType == 'V') {
    if (variableIndex < V_memory_count) {
      return String(V[variableIndex]);
      //      sprintf(S, "%.6f", V[variableIndex]); //увеличенее до 6 знаков после запятой
      //      return  S;   // return the value of the arduino V memory array
    }
  }

  else if (variableType == 'T') {
    if (variableIndex == 0) return T_0;
    else if (variableIndex == 1) return T_1;
    else if (variableIndex == 2) return T_2;
    //else if (variableIndex==3) return T_3;
  }

  return "";
}
//================================================================= onRequested









#endif
