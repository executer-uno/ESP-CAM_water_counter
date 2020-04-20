/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/
*********/

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include "fb_gfx.h"
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems
#include "esp_http_server.h"

#include "TFT_22_ILI9225.h"
#include <SPI.h>
// Include font definition files
// NOTE: These files may not have all characters defined! Check the GFXfont def
// params 3 + 4, e.g. 0x20 = 32 = space to 0x7E = 126 = ~

#include "fonts/FreeSansBold24pt7b.h"

#define TFT_CS         15
#define TFT_RST        13 // -1
#define TFT_RS         2 //RS 
#define TFT_SDI 12  // Data out SDA MOSI SDI 
#define TFT_CLK 14  // Clock out CLK

TFT_22_ILI9225 tft = TFT_22_ILI9225(TFT_RST, TFT_RS, TFT_CS, TFT_SDI, TFT_CLK, 0, 200);

#include "virtuino_pins.h"

#include "time.h"

#include "Credentials.h"

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7200;
const int   daylightOffset_sec = 3600; //летнее время 3600;

struct tm timeinfo; //структура времени записи кольцевого буфера

#define info_first 2 ////строка вывода результатов ширины и высоты цифр
#define info_result 58 //строка вывода результатов распознавания
#define info_gistorgamm 105 //строка вывода гистограммы
#define info_Hemming 130 //строка вывода информационнии о расстоянии Хемминга
#define info_frequency 145 //строка вывода информационнии о частоте совпадения
#define info_britnes 115 //строка вывода информационнии об уровнях яркости
#define info_time 160 //строка вывода на дисплей м3/мин сек и времени

//++++++++++++++++++++++++++++++++++++++++++ снизить до 50
#define Hemming_level 80 //Значенее расстояния Хемминга которое считается максимально допустимым при распознавании  50

#define width_letter 26 //ширина цифр в пикселях
#define number_letter 8 //число цифр в шкале
#define height_letter 26 //высота по y - совпадает с высотой эталона

#define average_count 10 //количество усреднений результатов распознавания
#define average_count_level average_count-3 //число усреднений, которое принимается за положительное при распознавании

#define F_HEIGHT 176 //высота обработки изображения совпадает с высотой дисплея 2.0
#define F_WIDTH  320 //ширина обработки изображения совпадает с шириной изображения камеры

#include "sample.h" //образцы эталонов

uint16_t Y_first, Y_last; //положение окна расположения цифр в буфере камеры

uint16_t pixel_level = 0; //пороговый уровень пикселей на изображении определеный методом Отцу

uint16_t max_letter_x[number_letter]; //массив середины цифры по оси Х

uint32_t l_32[number_letter][F_HEIGHT]; //массив после перевода распознаваемых цифр в 32 битное число. Запас по высоте равен высоте экрана
uint8_t result[average_count][number_letter]; //накопление результатов распознавания цифр со шкалы

uint16_t *frame_buf; //указатель на буфер для накопления кадров камеры

#define max_shift 9*3 //число вариантов сдвига перемещения эталона
int shift_XY[max_shift][2] = { //содержит сдвиг по оси X Y
  {0, 0},
  {0, 1},   //up
  {0, 2},   //up
  {0, 3},   //up
  {0, 4},   //up
  {0, -1},  //down
  {0, -2},  //down
  {0, -3},  //down
  {0, -4},  //down
  {1, 0},   //right
  {1, 1},   //right up
  {1, 2},   //right up
  {1, 3},   //right up
  {1, 4},   //right up
  {1, -1},  //right down
  {1, -2},  //right down
  {1, -3},  //right down
  {1, -4},  //right down
  { -1, 0}, //left
  { -1, 1}, //left up
  { -1, 2}, //left up
  { -1, 3}, //left up
  { -1, 4}, //left up
  { -1, -1},//left down
  { -1, -2},//left down
  { -1, -3},//left down
  { -1, -4},//left down
};

struct Hemming_struct { //структура расстояний Хемминга для всех цифр шкалы
  uint8_t result; // опознанная цифра
  uint16_t min_Hemming; // расстояния Хемминга для опознанной цифры
  uint8_t etalon_number; // номер эталона в массиве эталонов
  uint8_t frequency; //число совпадений при опознании
  uint8_t next_result; // значенее следующей цифры
  uint16_t next_min_Hemming; // расстояния Хемминга для следующей цифры
  uint8_t dig_defined; //набор символов, который определяется автоматически после распознавания
  uint8_t britnes_digital; //яркость для каждой буквы
  uint16_t x_width; //определенная ширина цифры
} Hemming[number_letter];

uint8_t frequency[number_of_samples][number_letter]; //подсчет максимального числа совпадений результатов распознавания

uint32_t used_samples[number_of_samples][number_letter]; //частота использования эталона

camera_fb_t * fb; //для работы камеры указатель на структуру буфер
sensor_t * s; //для работы камеры указаитель на структуру сенсора

// Replace with your network credentials
const char* ssid 		= MY_WIFI;
const char* password 	= MY_PASS;

#define PART_BOUNDARY "123456789000000000000987654321"

// This project was tested with the AI Thinker Model, M5STACK PSRAM Model and M5STACK WITHOUT PSRAM
#define CAMERA_MODEL_AI_THINKER

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22


const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
const char* _STREAM_BOUNDARY = "\n--" PART_BOUNDARY "\n";
const char* _STREAM_PART = "Content-Type: image/jpeg\nContent-Length: %u\n\n";

httpd_handle_t stream_httpd = NULL;

#include <Ticker.h> //esp32 library that calls functions periodically

Ticker Gas_minute_Ticker; //используется для расчета объма газа каждую минуту

#define size_m3 2048 //размер кольцевого буфера для хранения данных каждую минуту должен быть 256, 512, 1024 ...

//структура для сохранения информации о расчете объема газа
struct Gas_struct {
  uint32_t m3; //значенее объма газа умноженное на 100
  uint32_t minutes; //разница в минутах между предыдущим и текущим измерением
} Gas[size_m3];

uint16_t position_m3 = 0; //позиция сохранения данных

int offset_y_current; //текущее дополнительное смещение по оси Y

//---------------------------------------------------- m3_calculate
void m3_calculate() {

  uint32_t k = 1000000; //коэффициент перевода цифр шкалы в число
  uint32_t current_m3 = 0; //текущее значенее
  bool flag = true; //флаг рапознаввания

  uint16_t pos_1 = (position_m3 - 1) & (size_m3 - 1); //позиция в буфере места записи минус 1 - педыдущее значение

  for (uint8_t dig = 0; dig < number_letter - 1; dig++) { //проверка на все кроме последней цифры -1
    if ((Hemming[dig].frequency < average_count_level) || (Hemming[dig].min_Hemming > Hemming_level) || Hemming[dig].dig_defined == 10) {

      //нет правильного результата увеличим пропущенное время если это не начало и уже было распознано предыдущее значение м3 не равно 0
      if (Gas[pos_1].m3 != 0) Gas[position_m3].minutes++;

      current_m3 = 0; //обнуляем, т.к. нет правильного результата
      flag = false; //сбросим флаг - не распознали
      break; //выйти из цикла
    }
    current_m3 += Hemming[dig].dig_defined * k; //берем опознанное значенее и переводим в число
    k = k / 10; //уменьшаем коэффициент для следующего числа
  }
  Serial.printf("flag=%d current_m3= %d minutes=%d position_m3=%d pos_1=%d minutes_1=%d\n",
                flag, current_m3, Gas[position_m3].minutes, position_m3, pos_1, Gas[pos_1].minutes);

  if (flag) { //распознали сохраняем
    if (((current_m3 - Gas[pos_1].m3 < 0) || (current_m3 - Gas[pos_1].m3 > 6)) && (Gas[pos_1].m3 != 0)) { //ошибка распознавания вне пределов
      //если разница между предыдущем и текущим значением меньше нуля - текущее определили не верно
      //если разница больше 0,06 - ошибка определения - текущее значение определили не верно - за 1 минуту не может быть больше 0,05
      //при начальном заполнении буфера первый раз будет давать ошибку

      V[V_error_recognition]++; //увеличим счетчик ошибок неправльно распознанных
      Serial.printf("Значение %d м3 вне пределов на минуте %d\n", current_m3, Gas[pos_1].minutes);
    }
    else { //значения совпадают или нормально измененные
      V[V_error_recognition] = 0.0; //сбросим счетчик ошибок неверно распознанных
    }

    Serial.printf("Предыдущее %d текущее %d м3 позиция %d минут %d\tразница=%d\n", Gas[pos_1].m3, current_m3, position_m3, Gas[pos_1].minutes, current_m3 - Gas[pos_1].m3);

    if (Gas[pos_1].m3 == current_m3) {
      //если совпало с предыдущим просто увеличим время простоя на 1 минуту или на время пропущенных минут
      //если ранее были нераспознанные минуты, то прибавим к пропущенному времени
      //      Serial.printf("Совпадение значений current_m3 %.2f м3 увеличим предыдущее время Gas[pos_1].minutes=%d разница %f\n",current_m3, Gas[pos_1].minutes,current_m3 - Gas[pos_1].m3);
      Gas[pos_1].minutes += Gas[position_m3].minutes;
      Gas[position_m3].minutes = 1; //возобновим подсчет времени сначала для следующего элемента
      V[V_m3] = current_m3;
      V[V_m3_minutes] = Gas[pos_1].minutes;
    }
    else { //не совпали значения - есть изменения сохраним не обращая внимание на пределы и переход к следующему элементу
      Gas[position_m3].m3 = current_m3;
      V[V_m3] = current_m3;
      V[V_m3_minutes] = Gas[position_m3].minutes;

      position_m3 = (position_m3 + 1) & (size_m3 - 1); //переход к следующему элементу с учетом конца буффера;
    }

    Gas[position_m3].minutes = 1; //подсчет времени сначала для текущего элемента
    Gas[position_m3].m3 = 0;
  } //распознали сохраняем

  pos_1 = (position_m3 - 1) & (size_m3 - 1); //позиция в буфере места записи минус 1 - педыдущее значенее
  uint16_t pos_2 = (position_m3 - 2) & (size_m3 - 1); //позиция в буфере места записи минус 2

  if (Gas[pos_2].minutes != 0) { //если уже сохранено не менее 2-х элементов
    Serial.printf("Gas[pos_2].m3= %d Gas[pos_1].m3= %d minutes=%d difference_m3=%d position-1=%d position-2=%d\n",
                  Gas[pos_2].m3, Gas[pos_1].m3, Gas[pos_1].minutes, (Gas[pos_1].m3 - Gas[pos_2].m3) / Gas[pos_1].minutes, pos_1, pos_2);
  }
  else
    Serial.printf("current_m3= %d minutes=%d\n", Gas[pos_1].m3, Gas[pos_1].minutes);
}
//---------------------------------------------------- m3_calculate

//---------------------------------------------------- print_m3
void print_m3() {
  //вывести на монитор накопленные результаты m3
  time_t now;
  uint32_t all_minutes = 0; //для подсчета времени с начала отсчета

  uint16_t pos_1 = (position_m3 - 1) & (size_m3 - 1); //позиция в буфере места записи минус 1 - педыдущее значенее

  if (Gas[pos_1].minutes == 0) return; //не выводить если начало - нет предыдущего значения
  Serial.printf("i\tm3*100\tminutes\t\tmax number=%d\n", position_m3);

  for (uint32_t i = 0; i < size_m3; i++) {
    uint16_t pos = (position_m3 + i) & (size_m3 - 1); //позиция в буфере после места записи
    if (Gas[pos].m3 == 0) continue; //если обнаружили конец буфера продолжим
    //     Serial.printf("i=%d pos=%d position_m3+i=%d Gas[pos].minutes=%d Gas[pos].m3=%.2f\n",i, pos , position_m3+i,Gas[pos].minutes,Gas[pos].m3);
    all_minutes += Gas[pos].minutes;   //подсчет общего времени суммируем все
    Serial.printf("%d\t%d\t%d\n", pos, Gas[pos].m3, Gas[pos].minutes);
  }
  V[V_SH_M3] = 0; //выведем один раз

  struct tm timeinfo1; //структура времени записи кольцевого буфера

  if (!getLocalTime(&timeinfo1)) { //получим время записи сохранения м3
    Serial.printf("Failed to obtain time\n");
  }
  Serial.printf("\nСоздано  %02d.%02d.%4d %02d:%02d:%02d\n", timeinfo1.tm_mday, timeinfo1.tm_mon + 1, timeinfo1.tm_year + 1900,
                timeinfo1.tm_hour, timeinfo1.tm_min, timeinfo1.tm_sec);
  time(&now);
  now -= all_minutes * 60; //отнимим прошедшие минуты о получим начало отсчета
  localtime_r(&now, &timeinfo1);
  Serial.printf("Начало записи %02d.%02d.%4d %02d:%02d:%02d\n", timeinfo1.tm_mday, timeinfo1.tm_mon + 1, timeinfo1.tm_year + 1900,
                timeinfo1.tm_hour, timeinfo1.tm_min, timeinfo1.tm_sec);

  Serial.printf("\nСтатистика использования эталонов:\n");
  for(uint8_t dig = 0; dig < number_letter; dig++){
    Serial.printf("Позиция в шкале %d опеределено как цифра %d\n",dig,Hemming[dig].result);
    for(uint8_t i = 0; i < number_of_samples; i++){
      if(used_samples[i][dig] != 0)
        Serial.printf("Цифра %d\tномер эталона %02d\tопознан %d раз\n",sample_equation[i],i,used_samples[i][dig]);
    }
  }
  Serial.printf("\n");
}
//---------------------------------------------------- print_m3


//---------------------------------------------------- printBinary
#include <limits.h>

template<typename T>

void printBinary(T value, String s) {
  for ( size_t mask = 1 << ((sizeof(value) * CHAR_BIT) - 1); mask; mask >>= 1 ) {
    Serial.printf("%c", value & mask ? '1' : ' ');
  }
  Serial.printf("%s", s);
}
//---------------------------------------------------- printBinary


//---------------------------------------------------- britnes_digital
uint16_t find_middle_britnes_digital(uint16_t *fr_buf, bool show) {
  //расчет средней яркости пикселей для каждой цифры после того как определили их место
  for (uint8_t dig = 0; dig < number_letter; dig++) { //поочередно обрабатываем каждое знакоместо отельно
    float britnes  = 0;

    uint16_t w_l = width_letter;
    int x1 = max_letter_x[dig] - width_letter / 2; //для первой цифры размер может быть меньше установленной ширины
    if (x1 < 0) {
      w_l += x1; //x1 имеет отрицательное значенее поэтому суммируем
      if (show) Serial.printf("x1=%d max_letter_x[dig]=%d w_l=%d\n", x1, max_letter_x[dig], w_l);
      x1 = 0;
    }
    for (uint16_t y = Y_first; y < Y_last; y++) { //перебор по столбцу в пределах высоты буквы
      for (uint16_t x = x1; x < max_letter_x[dig] + width_letter / 2; x++) { //перебор по строке в пределах ширины одной цифры
        uint32_t i = (y * F_WIDTH + x);
        britnes += fr_buf[i]; //суммируем все значения
      }
    }
    Hemming[dig].britnes_digital = (int)(britnes / (w_l * (Y_last - Y_first))); //для первой цифры ширина может быть меньше
    if (show)
      Serial.printf("dig=%d britnes=%d pixel_level=%d\n", dig, (int)(britnes / (width_letter * (Y_last - Y_first))), pixel_level);
  }
}
//---------------------------------------------------- britnes_digital


//---------------------------------------------------- find_middle_level_image
uint16_t find_middle_level_image(uint16_t *fr_buf, bool show) {
  //найти уровень яркости изображения по Отцу

  float av = 0;
  uint16_t min1 = fr_buf[0];
  uint16_t max1 = fr_buf[0];
  uint32_t f_size = F_WIDTH * F_HEIGHT;

  //найти средний уровень пикселей окно табло - засвечено
  // Посчитаем минимальную и максимальную яркость всех пикселей
  for (uint32_t i = 0; i < f_size; i++) {
    av += fr_buf[i];
    if (fr_buf[i] < min1) min1 = fr_buf[i];
    if (fr_buf[i] > max1) max1 = fr_buf[i];
  }
  av = av / f_size;

  // Гистограмма будет ограничена снизу и сверху значениями min и max,
  // поэтому нет смысла создавать гистограмму размером 256 бинов
  int histSize = max1 - min1 + 1;

  int *hist = (int *) heap_caps_calloc(histSize * sizeof(int), 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (hist == NULL) {
    Serial.printf("Problem with heap_caps_malloc find_middle_level_y\n");
    return 0;
  }

  // Заполним гистограмму нулями
  for (int t = 0; t < histSize; t++)
    hist[t] = 0;

  // И вычислим высоту бинов
  for (int i = 0; i < f_size; i++)
    hist[fr_buf[i] - min1]++;

  // Введем два вспомогательных числа:
  int m = 0; // m - сумма высот всех бинов, домноженных на положение их середины
  int n = 0; // n - сумма высот всех бинов
  for (int t = 0; t <= max1 - min1; t++)
  {
    m += t * hist[t];
    n += hist[t];
  }

  float maxSigma = -1; // Максимальное значение межклассовой дисперсии
  int threshold = 0; // Порог, соответствующий maxSigma

  int alpha1 = 0; // Сумма высот всех бинов для класса 1
  int beta1 = 0; // Сумма высот всех бинов для класса 1, домноженных на положение их середины

  // Переменная alpha2 не нужна, т.к. она равна m - alpha1
  // Переменная beta2 не нужна, т.к. она равна n - alpha1

  // t пробегается по всем возможным значениям порога
  for (int t = 0; t < max1 - min1; t++)
  {
    alpha1 += t * hist[t];
    beta1 += hist[t];

    // Считаем вероятность класса 1.
    float w1 = (float)beta1 / n;
    // Нетрудно догадаться, что w2 тоже не нужна, т.к. она равна 1 - w1

    // a = a1 - a2, где a1, a2 - средние арифметические для классов 1 и 2
    float a = (float)alpha1 / beta1 - (float)(m - alpha1) / (n - beta1);

    // Наконец, считаем sigma
    float sigma = w1 * (1 - w1) * a * a;

    // Если sigma больше текущей максимальной, то обновляем maxSigma и порог
    if (sigma > maxSigma)
    {
      maxSigma = sigma;
      threshold = t;
    }
  }

  // Не забудем, что порог отсчитывался от min, а не от нуля
  threshold += min1;

  heap_caps_free(hist); //освободить буфер

  if (show)
    Serial.printf("min =%d max=%d cреднее по уровню=%.0f threshold= %d\n", min1, max1, av, threshold);
  // Все, порог посчитан, возвращаем его наверх :)
  return (uint16_t)(threshold);
}
//---------------------------------------------------- find_middle_level_image


//---------------------------------------------------- find_digits_y
void find_digits_y (uint16_t *fr_buf, uint16_t mid_level, uint8_t add_mid_level, bool show) {
  //fr_buf буфер с изображением формата uint16_t
  //mid_level средний уровень яркости изображения
  //add_mid_level повышение уровня для устранения засветки при анализе
  //show вывести информацию на экран

  //поиск положения цифр по высоте Y
#define  Y_FIRST_LOOK 10  //ограничим начало поиска 10 пикселями в строке
#define  Y_LAST_LOOK 100 //ограничим конец поиска 100 пикселями в строке
  Y_first = 0;
  Y_last = 0;
  float av = 0;
  char buf[50]; //буфер для перевода значений строку и печати на дисплеи

  /*
    //проверка на максимум уровня не должен быть больше 255
    if (mid_level + add_mid_level > 255)
      add_mid_level = (255 - mid_level);
  */
  //поиск среднего уровня
  for (uint8_t y = Y_FIRST_LOOK; y < Y_LAST_LOOK; y++) { //только в пределах экрана по высоте 10-100 строки
    for (uint16_t x = 0; x < tft.maxX(); x++) { //ограничим шириной экрана, а не всем изображением F_WIDTH
      uint32_t i = (y * F_WIDTH + x);
      if (fr_buf[i] > mid_level + add_mid_level) av++;
    }
  }
  av = (uint16_t) (av / (Y_LAST_LOOK - Y_FIRST_LOOK));

  for (uint8_t y = Y_FIRST_LOOK; y < Y_LAST_LOOK; y++) { //только в пределах экрана по высоте 10-100 строки
    float av1 = 0;
    for (uint16_t x = 0; x < tft.maxX(); x++) { //ограничим шириной экрана, а не всем изображением F_WIDTH
      uint32_t i = (y * F_WIDTH + x);
      if (fr_buf[i] > mid_level + add_mid_level) av1++;
    }
    if (av < av1) {
      if (show) {
        Serial.printf("av=%.0f av1=%.0f Y = %d\n", av, av1, y);
      }
      if (Y_first == 0) Y_first = y;
      Y_last = y;
    }
  }


  uint8_t Y_mid = Y_first + ((Y_last - Y_first) >> 1);
  if (Y_last - Y_first != height_letter) {
    Y_last  = Y_mid + (height_letter >> 1);
    Y_first = Y_mid - (height_letter >> 1);
  }

  tft.drawLine(0, Y_first, tft.maxX() - 1, Y_first, COLOR_YELLOW);
  tft.drawLine(0, Y_last, tft.maxX() - 1, Y_last, COLOR_YELLOW);
  tft.drawLine(0, Y_mid, tft.maxX() - 1, Y_mid, COLOR_CYAN);



  /*
    //вывод на дисплей индивидуальных значений яркости
      tft.setFont(Terminal6x8 ); //10 pixel
      tft.fillRectangle (0, info_britnes, tft.maxX(), 20, COLOR_BLACK); //очистить часть экрана

      tft.setColor(COLOR_WHITE);
      tft.setCursor(0, info_britnes);
      for (uint8_t dig = 0; dig < number_letter; dig++) {//вывести значения яркости
        tft.drawTextf("%d ", Hemming[dig].britnes_digital);
      }

  */

  //вывод на дисплей результатов сохраненных значений
  tft.setFont(Terminal6x8); //10 pixel
  tft.fillRectangle (0, info_time - 2, tft.maxX(), info_time + 10, COLOR_BLACK); //очистить часть экрана

  uint8_t pos_1 = (position_m3 - 1) & (size_m3 - 1); //позиция в буфере после места записи -1
  uint8_t pos_2 = (position_m3 - 2) & (size_m3 - 1); //позиция в буфере после места записи -2

  if (Gas[pos_2].minutes != 0) { //если уже сохранено не менее 2-х элементов
    sprintf(buf, "%4d mins %4.2f m3/m\0", Gas[pos_1].minutes, (Gas[pos_1].m3 - Gas[pos_2].m3) / (Gas[pos_1].minutes * 100.0));
    tft.drawText(0, info_time, buf, COLOR_WHITE);

    V[V_m3_m] = (Gas[pos_1].m3 - Gas[pos_2].m3) / (Gas[pos_1].minutes * 100.0);
    if (V[V_m3_m] > 1) V[V_m3_m] = 0; //за 1 минуту не может быть больше 1 м3
  }
  else {
    sprintf(buf, "%4d mins %4.2f m3\0", Gas[pos_1].minutes, Gas[pos_1].m3 / 100.0);
    tft.drawText(0, info_time, buf, COLOR_WHITE);
    V[V_m3_m] = 0;
  }

  sprintf(buf, "%02d:%02d:%02d\0", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  if (!getLocalTime(&timeinfo)) {
    Serial.printf("Failed to obtain time\n");
    tft.drawText(tft.maxX() - tft.getTextWidth(buf), info_time, buf, COLOR_RED); //9 * tft.getCharWidth(48)
  }
  else {
    sprintf(buf, "%02d:%02d:%02d\0", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    tft.drawText(tft.maxX() - tft.getTextWidth(buf), info_time, buf, COLOR_YELLOW);
  }

  uint16_t x_width_min = F_WIDTH; //ширина экрана
  uint16_t x_width_max = 0; //минимальное значенее

  for (uint8_t dig = 0; dig < number_letter; dig++) {
    //поиск максимальной и минимальной ширины определнной цифры
    if (x_width_min > Hemming[dig].x_width) x_width_min = Hemming[dig].x_width;
    if (x_width_max < Hemming[dig].x_width) x_width_max = Hemming[dig].x_width;
  }

  uint16_t next_x = 0;

  sprintf(buf, "Y_m=%2d ", Y_mid);
  if ((Y_last - Y_first) != sample_height)
    tft.drawText(next_x, info_first, buf, COLOR_YELLOW);
  else
    tft.drawText(next_x, info_first, buf, COLOR_GREEN);
  next_x += tft.getTextWidth(buf);

  sprintf(buf, " X_W =");
  tft.drawText(next_x, info_first, buf, COLOR_GREEN);
  next_x += tft.getTextWidth(buf);

  sprintf(buf, "%2d", x_width_min);
  if ((x_width_min < 3) || (x_width_min > width_letter)) //ширина не должна быть меньше 3 пикселей
    tft.drawText(next_x, info_first, buf, COLOR_YELLOW);
  else
    tft.drawText(next_x, info_first, buf, COLOR_GREEN);
  next_x += tft.getTextWidth(buf);

  sprintf(buf, "-%2d ", x_width_max);
  if ((x_width_max > width_letter) || (x_width_max <= x_width_min)) //ширина не должна быть больше ширины буквы
    tft.drawText(next_x, info_first, buf, COLOR_YELLOW);
  else
    tft.drawText(next_x, info_first, buf, COLOR_GREEN);
  next_x += tft.getTextWidth(buf);

  sprintf(buf, " Y_o=%.0f  %.0f", V[V_offset_y_test], V[V_offset_y_current]);
  tft.drawText(next_x, info_first, buf, COLOR_GREEN);
  next_x += tft.getTextWidth(buf);

  sprintf(buf, " %.0f", V[V_Sum_min_Hemming_current]);
  if (V[V_Sum_min_Hemming] < 160) //Суммарное значение Хемминга должно быть меньше 150
    tft.drawText(next_x, info_first, buf, COLOR_GREEN);
  else if (V[V_Sum_min_Hemming] > 250)
    tft.drawText(next_x, info_first, buf, COLOR_RED);
  else
    tft.drawText(next_x, info_first, buf, COLOR_YELLOW);


  if (show) {
    Serial.printf(" Y_first = %d Y_last = %d\n", Y_first, Y_last);
  }
}
//---------------------------------------------------- find_digits_y


//---------------------------------------------------- find_max_digital_X
void find_max_digital_X(uint16_t *fr_buf, uint16_t mid_level, uint8_t add_mid_level, bool show) {
  //fr_buf буфер с изображением формата uint16_t
  //mid_level средний уровень изображения
  //add_mid_level повышение следующего уровня для устранения засветки при отображении
  //show вывести информацию на экран

  //поиск границ цифр в найденной строке по X
  uint8_t letter[F_WIDTH]; //массив для поиска максимума по оси Х

  /*
    //проверка на максимум уровня
    if (mid_level + add_mid_level > 255)
      add_mid_level = (255 - mid_level);
  */
  //строим гистограмму подсчет количества единиц по столбцу
  for (uint16_t x = 0; x < F_WIDTH; x++) { //перебор по строке
    letter[x] = 0; //обнуляем начальное значение
    for (uint16_t y = Y_first; y < Y_last + 1; y++) { //ищем только в пределах обнаруженных цифр
      uint16_t i = (y * F_WIDTH + x);
      if (fr_buf[i] > mid_level + add_mid_level) {
        letter[x]++;
      }
    }
    //    if(show) Serial.printf("x=%3d letter=%d\n",x,letter[x]);
  }

  if (show) {
    //вывод гистограммы на дисплей с учетом смещения по оси Х
    tft.fillRectangle (0, info_Hemming - 30, tft.maxX(), info_Hemming - 2, COLOR_BLACK); //очистить часть экрана
    for (uint16_t x = V[V_offset_x_digital]; x < F_WIDTH; x++) { //перебор по строке  V[V_offset_x]
      if (letter[x] != 0)
        tft.drawLine(x - V[V_offset_x_digital], info_Hemming - 30, x - V[V_offset_x_digital], 100 + letter[x], COLOR_CYAN); //Y_last + 10 V[V_offset_x]
    }
  }

  //уточним центры цифр по гистограмме
  uint16_t x1 = 0, x2 = 0; //начало и конец цифры
  uint16_t  dig = 0; //номер найденной цифры от 0 до number_letter-1
  for (uint16_t x = 0; x < F_WIDTH; x++) { //перебор по строке
    if (letter[x] > 2) { //если есть пиксели найти начало больше 2 пикселей
      if (x1 != 0) x2 = x; //если было найдено начало установить конец
      else x1 = x; //установить начало
    }
    else { //нет пикселей
      if (x1 != 0 && x2 != 0) { //если были ранее определены пределы показать границы
        if (show) Serial.printf("x1=%4d x2=%4d x_mid=%4d d_x=%d dig=%d\n", x1, x2, ((x2 - x1) >> 1) + x1, (x2 - x1), dig);

        if (dig > number_letter - 1) { //усли найдено больше чем цифр в шкале 8 number_letter - 1
          if (show) Serial.printf("Найдено больще цифр чем в шкале по оси Х!!! %d\n", dig);
          return;
        }
        max_letter_x[dig] = ((x2 - x1) >> 1) + x1;

        Hemming[dig].x_width = (x2 - x1); //сохраним значенее ширины буквы

        dig++;
        //линия на +/- 5 пикселей от Y_last и Y_first
        tft.drawLine(((x2 - x1) >> 1) + x1 - V[V_offset_x_digital], Y_first - 5, ((x2 - x1) >> 1) + x1 - V[V_offset_x_digital], Y_last + 5, COLOR_BLUE);
        tft.drawLine(x1 - V[V_offset_x_digital], Y_first - 5, x1 - V[V_offset_x_digital], Y_last + 5, COLOR_OLIVE);
        tft.drawLine(x2 - V[V_offset_x_digital], Y_first - 5, x2 - V[V_offset_x_digital], Y_last + 5, COLOR_OLIVE);

        /*
                //построение диаграммы по оси Y между x1 и x2
                for (uint16_t y = Y_first; y < Y_last; y++) {
                  uint8_t y_l = 0;
                  for (uint16_t x = x1; x < x2; x++) {
                    uint16_t i = (y * F_WIDTH + x);
                    if (fr_buf[i] > mid_level + add_mid_level) {
                      y_l++;
                    }
                  }

                  //вывести гистограмму по высоте на экран
                  //          if (SH_0_1 == 1) {
                  //            for (uint8_t i = 0; i < y_l; i++)
                  //              Serial.printf("1");
                  //          }
                  ////          tft.drawFastHLine(x1 - V[V_offset_x_digital], Y_first - 65 + y, y_l, COLOR_YELLOW);
                  //          if (SH_0_1 == 1) Serial.printf("\n");
                }
                //        if (SH_0_1 == 1) Serial.printf("\n");
                //построение диаграммы по оси Y между x1 и x2
        */
      }

      //обнулим значение для следующей цифры
      x1 = 0;
      x2 = 0;
    }
  }
  if (dig != number_letter)
    if (show) Serial.printf("Не все цифры шкалы найдены по оси Х!!! %d\n", dig);
}
//---------------------------------------------------- find_max_digital_X


//---------------------------------------------------- sum_one
uint8_t sum_one(uint32_t d) { //суммирование всех 1 в 32 битном числе
  uint8_t r = 0;
  for (uint8_t i = 0; i < 32; i++) {
    r += d & 0x1;
    d = d >> 1;
  }
  return r;
}
//---------------------------------------------------- sum_one


//---------------------------------------------------- compare
uint8_t compare(uint8_t y, uint8_t samp_dig, uint8_t dig, int X_shift, int Y_shift, bool show) {
  //y положенее по оси Y
  //samp_dig номер эталона
  //dig - какую цифру обрабатываем
  //X_shift сдвиг по оси +/- Х
  //Y_shift сдвиг по оси +/- Y
  //show выводить на экран

  uint32_t samp = sample[samp_dig][y];//центры эталона и сравниваемая цифра совпадают
  //<< 5; нужно подготовить центры середина эталона полученная из знакогенератора 8, середина цифры от камеры 13

  uint32_t samp1;

  if ((y + Y_shift < F_HEIGHT) || (y + Y_shift > 0)) {
    if (X_shift < 0)
      samp1 = l_32[dig][y + Y_shift] >> abs(X_shift); //образец который сравниваем может быть отрицательное значенее
    else
      samp1 = l_32[dig][y + Y_shift] << X_shift; //образец который сравниваем
  }
  else samp1 = 0;

  if (show)
    printBinary(samp1 ^ samp, "\t");
  return sum_one(samp1 ^ samp);
}
//---------------------------------------------------- compare


//---------------------------------------------------- image_recognition
uint8_t image_recognition(uint8_t dig, uint8_t dig_show) {
  //dig - какую цифру обрабатываем
  //dig_show какую цифру отображаем если > 8 = не выводим

  //распознавание цифр
  //сравнить с эталоном - расчет расстояния Хемминга
  uint8_t min_dig_number = number_of_samples; //присвоим максимальное значенее

  //вывод цифры со шкалы в двоичном виде
  if (dig == dig_show) {
    Serial.printf("------------------------------\n");
    for (uint8_t y = 0; y < Y_last - Y_first; y++) { //перебор по Y
      printBinary(l_32[dig][y], "\n");
    }
    Serial.printf("------------------------------\n");
  }

  //вывод цифры со шкалы в HEX формате
  if (V[V_SH_HEX] == 1) {
    Serial.printf("{//%d\n", dig);
    for (uint8_t y = 0; y < Y_last - Y_first; y++) { //перебор по Y
      Serial.printf("0x%08lx,\t//", l_32[dig][y]);
      printBinary(l_32[dig][y], "\n");
    }
    Serial.printf("},\n");
  }

  uint32_t min_Hemming[number_of_samples]; //массив минимальных расстояний Хемминга для всех эталонов

  for (uint8_t samp_dig = 0; samp_dig < number_of_samples; samp_dig++) { //перебор по все эталлонам
    uint16_t shift[max_shift]; //хранение результатов расчетов расстояния Хеминга после всех вариантов сдвигов

    for (uint8_t i = 0; i < max_shift; i++)
      shift[i] = 0; //обнулим для накопления результатов расчета расстояния Хеминга

    for (uint8_t y = 0; y < sample_height; y++) { //перебор по Y - высоте эталона и образца
      for (uint8_t i = 0; i < max_shift; i++) //перебор по всем сдвигам
        shift[i] += compare(y, samp_dig, dig, shift_XY[i][0], shift_XY[i][1], dig == dig_show);
      if (dig == dig_show) Serial.printf("\n");
    }

    if (dig == dig_show) {
      Serial.printf("Etalon Digit=%d", samp_dig);
      for (uint8_t i = 0; i < max_shift; i++)
        Serial.printf("\tshift=%3d %d %d\t\t\t", shift[i], shift_XY[i][0], shift_XY[i][1]);
      Serial.printf("\n");
    }

    //поиск минималного значения после сдвигов
    min_Hemming[samp_dig] = shift[0];
    for (uint8_t i = 0; i < max_shift; i++) {
      if (min_Hemming[samp_dig] > shift[i])
        min_Hemming[samp_dig] = shift[i];
    }
  }

  uint32_t min_dig = 1024; //будет содержать минимальное значение расстояния Хемминга

  for (uint8_t samp_dig = 0; samp_dig < number_of_samples; samp_dig++) { //перебор по все эталлонам
    if (min_dig >= min_Hemming[samp_dig]) {
      min_dig = min_Hemming[samp_dig];
      min_dig_number = sample_equation[samp_dig]; //получить цифру соответсвия
      Hemming[dig].etalon_number = samp_dig; //номер эталона в массиве
    }
    if (dig == dig_show)
      Serial.printf("Etalon=%d\tmin_Hemming=%d\n", sample_equation[samp_dig], min_Hemming[samp_dig]);
  }

  if (dig == dig_show)
    Serial.printf("\n***** Found Digit=%d\tmin_Hemming= %d *****\n", min_dig_number, min_dig);

  Hemming[dig].min_Hemming = min_dig;  //сохранить значениее расстояния Хемминга

  //поиск следующего минимума

  min_dig = 1024; //будет содержать минимальное значение расстояния Хемминга
  Hemming[dig].next_result = 10; //всего цифры от 0 до 9

  for (uint8_t samp_dig = 0; samp_dig < number_of_samples; samp_dig++) { //перебор по все эталлонам
    //    Serial.printf("samp_dig=%d min_dig_number=%d min_Hemming=%d min_dig=%d\n",samp_dig,min_dig_number,min_Hemming[samp_dig], min_dig);
    if (sample_equation[samp_dig] == min_dig_number) continue; //если уже найденный минимум пропустить
    if (min_dig >= min_Hemming[samp_dig]) {
      min_dig = min_Hemming[samp_dig];
      Hemming[dig].next_result = sample_equation[samp_dig]; //значенее опознанной цифры

    }
  }

  Hemming[dig].next_min_Hemming = min_dig;  //сохранить значение расстояния Хемминга

  return min_dig_number;
}
//---------------------------------------------------- image_recognition


//---------------------------------------------------- convert_to_32
void convert_to_32(uint16_t *fr_buf, uint16_t mid_level, uint8_t add_mid_level, bool show) {

  /*
    //проверка на максимум уровня
    if (mid_level + add_mid_level > 255)
      add_mid_level = (255 - mid_level);
  */

  for (uint8_t dig = 0; dig < number_letter; dig++) { //всего 8 цифр в шкале последовательно обрабатываем каждую
    for (uint16_t y = Y_first; y < Y_last; y++) { //перебор по столбцу
      l_32[dig][y - Y_first] = 0;
      int x1 = max_letter_x[dig] - width_letter / 2; //для первой цифры размер может быть меньше установленной ширины
      if (x1 < 0) x1 = 0;
      for (uint16_t x = x1; x < max_letter_x[dig] + width_letter / 2; x++) { //перебор по строке в пределах одной цифры
        l_32[dig][y - Y_first] = l_32[dig][y - Y_first] << 1; //сдвиг на 1 позицию
        uint32_t i = (y * F_WIDTH + x);

        if (fr_buf[i] > Hemming[dig].britnes_digital + add_mid_level) { //индивидуальный уровень яркости для каждой цифры
          //        if (fr_buf[i] > mid_level + add_mid_level) { //средний уровень
          if (show || V[V_SH_0_1] != 0) Serial.printf("1");
          l_32[dig][y - Y_first]++;
        }
        else {
          if (show) Serial.printf(" ");
          if (V[V_SH_0_1] == 1) Serial.printf(" ");
          if (V[V_SH_0_1] == 2) Serial.printf("0");
        }
      }
      if (show) Serial.printf("|0x%08lx\n", l_32[dig][y - Y_first]);
      if (V[V_SH_0_1] != 0) Serial.printf("\n");
    }
    if (show) Serial.printf("Letter box middel = %d d_x = %d d_y =%d mid_line_y=%d\n", max_letter_x[dig], width_letter, Y_last - Y_first, Y_first + (Y_last - Y_first) / 2);
    if (V[V_SH_0_1] != 0) Serial.printf("\n");
  }
}
//---------------------------------------------------- convert_to_32


//---------------------------------------------------- dispalay_ttf_B_W
esp_err_t dispalay_ttf_B_W(uint16_t *fr_buf, uint16_t mid_level, uint8_t add_mid_level) {
  //fr_buf буфер с изображением формата uint16_t
  //X0 начальная кордината вывода по оси Х
  //Y0 начальная кордината вывода по оси Y
  //mid_level средний уровень изображения применятся индивидуальный для каждой цифры
  //add_mid_level повышение следующего уровня для устранения засветки при отображении



  //зарезервировтать паять для буфера дисплея
  uint16_t W = tft.maxX();
  uint16_t H = tft.maxY();

  uint16_t *disp_buf = (uint16_t *)heap_caps_calloc(W * H * 2, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (disp_buf == NULL) {
    Serial.printf("malloc failed f_b\n");
    return ESP_FAIL;
  }

  for (int y = 0; y < H; y++) { //выводим только по высоте экрана //tft.maxX() tft.maxY()
    for (int x = 0; x < W; x++) {
      uint8_t dig = 0; //номер цифры табло
      uint32_t i = (y * F_WIDTH + x + V[V_offset_x_digital]);
      uint32_t j = (y * W + x);
      uint16_t color;
      if (s->pixformat == PIXFORMAT_GRAYSCALE) { //если GRAYSCALE преобразовать в RGB565 каждый байт буфера
        color = (((uint16_t)(fr_buf[i]) & 0xF8) << 8) | (((uint16_t)(fr_buf[i]) & 0xFC) << 3) | ((uint16_t)(fr_buf[i]) >> 3);
      }
      else
        color = fr_buf[i];

      if (V[V_GBW] == 0) {
        if (x > max_letter_x[dig] + width_letter / 2) { //если текущее значенее в пределах цифры, то использовать соответствующее значенее яркости
          dig++; //перейти к следующей цифре
          if (dig > number_letter) dig = number_letter - 1; //если больше цифр то принимать яркость последней
        }
        if (fr_buf[i] < Hemming[dig].britnes_digital + add_mid_level) //индивидуальный уровень яркости для каждой цифры
          *(disp_buf + j)  = COLOR_BLACK;
        else
          *(disp_buf + j) = COLOR_WHITE;
      }
      else *(disp_buf + j) = color;
    }

  }
  if (V[V_GBW] == 2) //если выводить полный экран
    tft.drawBitmap(0, 0, disp_buf, W, H); //отобразить на дисплеи
  else
    tft.drawBitmap(0, 0, disp_buf, W, V[V_level_Y_down] + 10); //отобразить на дисплеи часть изображения с запасом на 10 пикселей

  heap_caps_free(disp_buf); //освободить буфер

  return ESP_OK;
}
//---------------------------------------------------- dispalay_ttf_B_W


//---------------------------------------------------- sum_frames
esp_err_t sum_frames(uint16_t *fr_buf, bool show, uint8_t Y_up, uint8_t Y_down) {
  uint32_t tstart;
  fb = NULL;

  //накопление кадров - проинтегрировать несколько кадров для устранения шумов
  tstart = clock();

  memset(fr_buf, 0, F_WIDTH * F_HEIGHT * 2); //выделить память и очистить размер в байтих

  uint8_t frame_c = V[V_number_of_sum_frames];
  if (s->pixformat != PIXFORMAT_GRAYSCALE)
    frame_c = 1; //если цветное то не суммировать по кадрам

  for (uint8_t frames = 0; frames < frame_c; frames++) { //усредненее по кадрам frame_count
    if (fb) { //освободить буфер
      esp_camera_fb_return(fb);
      fb = NULL;
    }
    fb = esp_camera_fb_get(); //получить данные от камеры
    if (!fb) {
      Serial.printf("Camera capture failed to display\n");
      return ESP_FAIL;
    }

    uint32_t i_max = fb->height * fb->width; //максимальное значенее массива для данного экрана

    for (uint16_t y = 0; y < F_HEIGHT; y++) { //работаем только с верхней частью кадра
      for (uint16_t x = 0; x < fb->width; x++) {
        if (s->pixformat == PIXFORMAT_GRAYSCALE) {
          uint32_t i = ((y + V[V_offset_y] + offset_y_current) * fb->width + x + V[V_offset_x]); //смещенее по оси Y и Х
          uint32_t j = (y * fb->width + x); //GRAYSCALE
          if ((y < Y_up) || (y > Y_down)) {
            fr_buf[j] = 0; //обнулим значения ниже и выше Y - иммитация шторки
            //            Serial.printf("\n");
          }
          else {
            if (i < i_max) //размер экрана (176)+смещенее может быть больше размера изображения 240
              fr_buf[j] += fb->buf[i];
            else
              fr_buf[j] = 0;
          }
        }
        else { //если RGB565 берем 8 битный буфер и преобразуем в 16 битный
          uint32_t i = (y * fb->width + x) << 1; //если RGB565
          uint32_t j = (y * fb->width + x); //если RGB565
          //https://github.com/techtoys/SSD2805/blob/master/Microchip/Include/Graphics/gfxcolors.h
          fr_buf[j] += (uint16_t)(fb->buf[i]) << 8 | (uint16_t)(fb->buf[i + 1]); //преобразуем в 16 битное
        }
      }
    }
  } //суммирование по кадрам

  //усредним все пиксели
  for (uint16_t i = 0; i < F_WIDTH * F_HEIGHT; i++) {
    fr_buf[i] = (uint16_t)(fr_buf[i] / frame_c);
  }

  if (fb) { //освободить буфер
    esp_camera_fb_return(fb);
    fb = NULL;
  }
  if (show) {
    Serial.printf("Capture camera time: %u ms of %d frames\n", clock() - tstart, frame_c);
  }
  return ESP_OK;
}
//---------------------------------------------------- sum_frames


//---------------------------------------------------- camera_capture
esp_err_t camera_capture(uint16_t *fr_buf, bool show, uint8_t Y_up, uint8_t Y_down) {
  //сумировать кадры
  if (sum_frames(fr_buf, show, Y_up, Y_down) != ESP_OK) return ESP_FAIL;

  uint32_t tstart = clock();
  if (show) {
    dispalay_ttf_B_W(fr_buf, 0, 0);
    Serial.printf("Send buffer time: %u ms\n", clock() - tstart);
  }
}
//---------------------------------------------------- camera_capture


//---------------------------------------------------- setup
void setup() {

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  Serial.setDebugOutput(false);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  /*
    config.pixel_format = PIXFORMAT_GRAYSCALE; //PIXFORMAT_JPEG;

    //init with high specs to pre-allocate larger buffers
    if (psramFound()) {
      //    config.frame_size = FRAMESIZE_UXGA;
      config.frame_size = FRAMESIZE_QVGA;
      config.jpeg_quality = 10;
      config.fb_count = 1;
    } else {
      config.frame_size = FRAMESIZE_SCOLOR;
      config.jpeg_quality = 12;
      config.fb_count = 1;
    }

    //  config.frame_size = FRAMESIZE_QVGA;
    // Camera init
  */
  // for display

  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_GRAYSCALE; //PIXFORMAT_GRAYSCALE; //PIXFORMAT_RGB565;
  config.fb_count = 1; //2

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(1000);
    ESP.restart();
  }

  //drop down frame size for higher initial frame rate
  s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);

  /*
    #ifdef  display_2_0
    s->set_hmirror(s, 1);
    s->set_vflip(s, 1);
    #endif
  */
  //  s->set_special_effect(s, 1); //Negative
  //s->set_brightness(s, Value);
  //       s->set_saturation(s, Value);
  //       s->set_contrast(s, Value);

  WiFi_Connect();

  tft.begin();
  tft.setOrientation(3);
  tft.clear(); //черным

  uint32_t f8  = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  frame_buf = (uint16_t *)heap_caps_calloc(F_WIDTH * F_HEIGHT * 2, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (frame_buf == NULL) {
    Serial.printf("malloc failed frame_buf\n");
  }
  else {
    Serial.printf("malloc succeeded frame_buf\n");
  }
  Serial.printf("8BIT = %d\n", f8 - heap_caps_get_free_size(MALLOC_CAP_8BIT));

  for (uint8_t dig = 0; dig < number_letter; dig++) {
    Hemming[dig].dig_defined = 10; //заносим первоначально максимальное число вне диапазона 0-9
  }
  // Start web server
  //  startCameraServer();

  virtuino.begin(onReceived, onRequested, 512); //Start Virtuino. Set the buffer to 256. With this buffer Virtuino can control about 28 pins (1 command = 9bytes) The T(text) commands with 20 characters need 20+6 bytes
  //virtuino.key="1234";                       //This is the Virtuino password. Only requests the start with this key are accepted from the library
  // avoid special characters like ! $ = @ # % & * on your password. Use only numbers or text characters
  server.begin();

  for (uint16_t i = 0; i < size_m3; i++) { //обнулим буфер сохранения значений
    Gas[i].m3 = 0;
    Gas[i].minutes = 0;
  }
  Gas[0].minutes = 1; //подсчет времени сначала для текущего элемента

  Gas_minute_Ticker.attach(60, m3_calculate); //вызывать расчета объма газа каждую минуту 60

  //init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Set timezone to Ukraine EET
  setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1);

  if (!getLocalTime(&timeinfo)) { //получим время начала записи сохранения м3
    Serial.printf("Failed to obtain time\n");
    return;
  }

  EEPROM.begin(16);//Установить размер внутренней памяти для хранения первоначальных значений

  init_V(); //инициализация начальных данных
  
  for(uint8_t dig = 0; dig < number_letter; dig++) //обнулить частоту использования эталонов
    for(uint8_t i = 0; i < number_of_samples; i++) 
      used_samples[i][dig] = 0;
}
//---------------------------------------------------- setup

//---------------------------------------------------- WiFi_Connect
void WiFi_Connect()
{
  // Wi-Fi connection
  uint32_t timeout = millis();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.printf(".");
    if (millis() - timeout > 5000) break;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected.\nCamera Stream Ready! Go to: http://");
    Serial.printf("%s\n", WiFi.localIP().toString().c_str());
  }
  else { //create AP
    WiFi.softAP("ESP32", "87654321");
    Serial.printf("\nWiFi %s not found create AP Name - 'ESP32' Password - '87654321'\n", ssid);
    Serial.printf("Camera Stream Ready! Go to: http://");
    Serial.printf("%s\n", WiFi.softAPIP().toString());
  }

  if (WiFi.getAutoConnect() != true)    //configuration will be saved into SDK flash area
  {
    Serial.printf("Set setAutoConnect and setAutoReconnect\n");
    WiFi.setAutoConnect(true);   //on power-on automatically connects to last used hwAP
    WiFi.setAutoReconnect(true);    //automatically reconnects to hwAP in case it's disconnected
  }
}
//---------------------------------------------------- WiFi_Connect

//---------------------------------------------------- find_max_number
uint8_t  find_max_number(uint8_t d) {
  //возврат максимального значения из всех найденых
  uint8_t n = 0; //первое значенее 0
  uint8_t m1 = frequency[0][d]; //присваиваем первое значение
  for (uint8_t i = 1; i < number_of_samples; i++) {
    if (m1 < frequency[i][d]) {
      m1 = frequency[i][d];
      n = i;
    }
  }
  return n;
}
//---------------------------------------------------- find_max_number


//---------------------------------------------------- show_result
void show_result(bool show) {
  uint8_t defined;
  uint16_t next_x = 0;
  char buf[10]; //буфер для формирования строки расстояния Хемминга и частоты повторения цифр

  T_0 = ""; //обновим строчку вывода результатов

  int16_t w, h;

  tft.setGFXFont(&FreeSansBold24pt7b); // Set current font
  tft.getGFXTextExtent("0", 0, info_result, &w, &h); // Get string extents
  h += info_result;
  //  Serial.printf("info_result=%d w=%d h=%d\n",info_result,w,h);

  //  tft.setFont(Trebuchet_MS16x21); //22 pixel for size 3 Trebuchet_MS16x21

  tft.fillRectangle (0, info_result - 2, tft.maxX(), h + 7, COLOR_BLACK); //очистить часть экрана GFXFont привязан верхней точкой

  //найти максимальную частоту вхождения цифр после опознавания
  for (uint8_t dig = 0; dig < number_letter; dig++) { //number_letter
    defined = find_max_number(dig);
    sprintf(buf, "%d\0", defined);
    //обновленее первоначально предопределенного набора символов если частота определения символа более 7 и рассояние Хемминга менее Hemming_level
    if ((frequency[defined][dig] > average_count_level) && (Hemming[dig].min_Hemming < Hemming_level)) {
      if (defined != Hemming[dig].dig_defined) { //корректно обнаружили первый раз
        //        Serial.printf("Change defined digital in position=%d from=%d to %d\n", dig, Hemming[dig].dig_defined, defined);
        Hemming[dig].dig_defined = defined;
        tft.drawGFXText(next_x, h, buf, COLOR_YELLOW); // Print string
        //        tft.drawText(next_x, info_result,buf,COLOR_YELLOW); //цифра опознана с большой вероятностью
      }
      else tft.drawGFXText(next_x, h, buf, COLOR_GREEN); //цифра распознана корректно уже неоднократно
    }
    else tft.drawGFXText(next_x, h, buf, COLOR_RED);

    T_0 += defined;
    next_x += w; //шаг между цифрами

    if (dig == 4) {
      T_0 += ".";
      tft.drawGFXText(next_x, h, ".", COLOR_GREEN);
      next_x += (w >> 1); //шаг между цифрами
    }


    Hemming[dig].result = defined;
    Hemming[dig].frequency = frequency[defined][dig];

    if ((Hemming[dig].frequency < average_count_level) && (Hemming[dig].min_Hemming > Hemming_level)) {
      Hemming[dig].dig_defined = 10; //не распознали с большой вероятностью
    }

    if (show)
      Serial.printf("found=%2d freq=%2d Hem_min=%4d Hem_defined =%d position=%2d Hem_next=%4d next_dig=%2d delta=%3d x_w=%d\n",
                    Hemming[dig].result, Hemming[dig].frequency, Hemming[dig].min_Hemming, Hemming[dig].dig_defined, Hemming[dig].etalon_number,
                    Hemming[dig].next_min_Hemming, Hemming[dig].next_result, Hemming[dig].next_min_Hemming - Hemming[dig].min_Hemming, Hemming[dig].x_width);

    used_samples[Hemming[dig].etalon_number][dig]++; //частота использования эталона
  }

  //сохраненее данных для вывода на экран Virtuino
  V[V_D0] =   Hemming[0].dig_defined;
  V[V_D1] =   Hemming[1].dig_defined;
  V[V_D2] =   Hemming[2].dig_defined;
  V[V_D3] =   Hemming[3].dig_defined;
  V[V_D4] =   Hemming[4].dig_defined;
  V[V_D5] =   Hemming[5].dig_defined;
  V[V_D6] =   Hemming[6].dig_defined;
  V[V_D7] =   Hemming[7].dig_defined;

  //вывод растояния Хемминга
  T_1 = "";
  T_2 = "";

  tft.setFont(Terminal6x8); //10 pixel
  tft.fillRectangle (0, info_Hemming - 2, tft.maxX(), info_Hemming + 24, COLOR_BLACK); //очистить часть экрана для расстояния Хеминга и частоты

  for (uint8_t dig = 0; dig < number_letter; dig++) {
    sprintf(buf, "|%3d \0", Hemming[dig].min_Hemming);
    next_x = max(tft.getTextWidth(T_1), tft.getTextWidth(T_2));
    if (next_x != 0) next_x -= tft.getTextWidth(" ");

    if (Hemming[dig].min_Hemming < Hemming_level)
      tft.drawText(next_x, info_Hemming, buf, COLOR_GREEN); //печатать каждую цифру со смещенеем на экране дисплея
    else
      tft.drawText(next_x, info_Hemming, buf, COLOR_RED); //печатать каждую цифру со смещенеем на экране дисплея

    //    Serial.printf("%3d %3d '%s'\n",next_x,tft.getTextWidth(T_1),T_1.c_str());
    T_1 += buf;

    sprintf(buf, "|%3d \0", Hemming[dig].frequency);
    if (Hemming[dig].frequency > average_count_level)
      tft.drawText(next_x, info_frequency, buf, COLOR_GREEN); //печатать каждую цифру со смещенеем на экране дисплея
    else
      tft.drawText(next_x, info_frequency, buf, COLOR_RED); //печатать каждую цифру со смещенеем на экране дисплея

    //    Serial.printf("%3d %3d %3d '%s'\n",next_x,tft.getTextWidth(T_2),tft.getTextWidth(T_2)-5,T_2.c_str());
    T_2 += buf;
  }
  T_1 += "|";
  T_2 += "|";
  //  Serial.printf("T_1 = %s\n",T_2.c_str());
}
//---------------------------------------------------- show_result

//uint32_t free_heap;
bool V_GBW_old = false;

//---------------------------------------------------- loop
void loop() {

#define min_max_offset_y_test 3 //значенее смещения +/-1 или 0
  uint16_t Sum_min_Hemming[min_max_offset_y_test]; //количество вариантов поиска смещения по оси Y для автоматической подстройки

  static uint8_t WiFi_Lost = 0; //счетчик потери связи WiFi

  for (uint8_t offset_y_test = 0; offset_y_test < min_max_offset_y_test; offset_y_test++) { //попробовать смещение по оси Y
    offset_y_current = V[V_offset_y_test] + (offset_y_test - 1); //к уже определенному ранее значению смещения попробовать новое смещение +/-1 или 0

    for (uint8_t dig = 0; dig < number_letter; dig++) { //обнулить массив для поиска частоты повторения цифр
      for (uint8_t i = 0; i < number_of_samples; i++) { //перебор по всем значения образцов
        frequency[i][dig] = 0;
      }
    }

    for (uint8_t count = 0; count < average_count; count++) { //повторим результат и найдем опознаные числа
      //обработка запросов web сервера Virtuino
      virtuinoRun();

      if (V[V_RESTART]) ESP.restart(); //если нажата клавиша перезагрузки в приложении - перегрузить. Пароль 1234

      if (V[V_SH_M3] == 1) print_m3(); //вывести накопленные даные на экран монитора

      if (V[V_GBW] == 2) { //Вывод полного экрана без анализа
        camera_capture(frame_buf, false, 0, F_HEIGHT); //получить кадры с камеры и усреднить их
        dispalay_ttf_B_W(frame_buf, pixel_level, V[V_level_dispalay_ttf_B_W]); //повысим на 5-20 единиц, чтобы убрать засветку
        V_GBW_old = true;
      }
      else {
        if (V_GBW_old) tft.clear(); //очистка после вывода полного экрана без анализа
        V_GBW_old = false;

        if (V[V_GBW] == 1)
          camera_capture(frame_buf, false, V[V_level_Y_up] - 10, V[V_level_Y_down] + 10); //получить кадры с камеры и усреднить их
        else
          camera_capture(frame_buf, false, V[V_level_Y_up], V[V_level_Y_down]); //получить кадры с камеры и усреднить их
        //найти средний уровень пикселей окна табло
        pixel_level = find_middle_level_image(frame_buf, false);

        //отображение на дисплеи
        dispalay_ttf_B_W(frame_buf, pixel_level, V[V_level_dispalay_ttf_B_W]); //повысим на 5-20 единиц, чтобы убрать засветку

        //  free_heap  = heap_caps_get_free_size(MALLOC_CAP_8BIT);
        //поиск положения окна цифр - при найденом уровне по оси y
        find_digits_y(frame_buf, pixel_level, V[V_level_find_digital_Y], false); //уровень повысим на 15 единиц, чтобы убрать засветку
        //  Serial.printf("heap = %d\n",free_heap-heap_caps_get_free_size(MALLOC_CAP_8BIT));

        //поиск максимума - предположительно середины цифр
        find_max_digital_X(frame_buf, pixel_level, V[V_level_find_digital_X], false); //уровень повысим на 7 единиц, чтобы убрать засветку

        //найти средний уровень для каждой цифры
        find_middle_britnes_digital(frame_buf, false);

        //преобразование в 32 битное числа
        convert_to_32(frame_buf, pixel_level, V[V_level_convert_to_32], false); //уровень повысим на 20 единиц, чтобы убрать засветку

        //сравнить с эталоном - рассчет расстояния Хемминга
        for (uint8_t dig = 0; dig < number_letter; dig++) { //проврека по всем цифрам шкалы
          result[count][dig] = image_recognition(dig, V[V_show_digital]);
          frequency[result[count][dig]][dig]++; //посчет числа совпадения цифра определенной цифры
        }
      }
    } //повторим результат и найдем опознаные числа

    if (V[V_SH_M3] == 1) print_m3(); //вывести накопленные даные на экран монитора

    if (V[V_GBW] != 2) {
      show_result(true);

      //суммарное значение расстояния Хемминга для всех цифр при разном смещении

      Sum_min_Hemming[offset_y_test] = 0; //обнулим для последующего накопления
      for (uint8_t dig = 0; dig < number_letter - 1; dig++) //без последней цифры
        Sum_min_Hemming[offset_y_test] += Hemming[dig].min_Hemming;
      V[V_Sum_min_Hemming_current] =  Sum_min_Hemming[offset_y_test]; //передадим текущее значение в приложение

      V[V_offset_y_current] = (offset_y_test - 1);

      Serial.printf("Суммарное расстояние Хемминга=%d при cмещении=%d итоговое смещение=%.0f\n\n", Sum_min_Hemming[offset_y_test], (offset_y_test - 1), V[V_offset_y_test]);
    }
    change_variables(false); //если были изменения коэффициентов записать
  } //попробовать смещение по оси Y

  //поиск минимального суммарного значения расстояния Хемминга для всех цифр при разном смещении
  uint16_t Sum_min = Sum_min_Hemming[0]; //минимальное значение расстояния Хемминга
  uint8_t Sum_min_offset_y_test = 0; //смещение по оси Y
  
  for (uint8_t offset_y_test = 0; offset_y_test < min_max_offset_y_test; offset_y_test++) { //попробовать смещение по оси Y
     if (Sum_min > Sum_min_Hemming[offset_y_test]) {
      Sum_min = Sum_min_Hemming[offset_y_test];
      Sum_min_offset_y_test = offset_y_test;
    }
  }

  if (V[V_GBW] != 2) {
    V[V_Sum_min_Hemming] =  Sum_min; //передадим текущее значение в приложение
    if (V[V_Sum_min_Hemming] > 250) { //суммарное значенее расстояния Хемминга не должно быть более 250
      V[V_Sum_min_Hemming_error]++;
      Serial.printf("Очевидно сбой суммарное расстояние Хемминга=%.0f всего ошибок=%.0f\n", V[V_Sum_min_Hemming], V[V_Sum_min_Hemming_error]);
    }
    else {
      V[V_offset_y_test] +=  (Sum_min_offset_y_test - 1); //запомним лучшее значение
    }

    Serial.printf("Результат суммарное расстояние Хемминга=%d при смещении=%d итоговое смещение=%.0f всего сбоев %.0f\n\n", Sum_min, Sum_min_offset_y_test - 1, V[V_offset_y_test], V[V_Sum_min_Hemming_error]);
  }

  if (WiFi.status() != WL_CONNECTED) {
    WiFi_Lost++;
    Serial.printf("Сеть потеряна %d\n", WiFi_Lost);
  }
  else WiFi_Lost = 0;
  if (WiFi_Lost == 6) WiFi_Connect(); //если нет связи около 4 минут пересоединиться
}
//---------------------------------------------------- loop
