/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-cam-video-streaming-web-server-camera-home-assistant/
*********/

// Look at that proj https://bitluni.net/esp32-i2s-camera-ov7670 for Display references
// ESP32 I2S Camera (OV7670)

// SPIFFS and ESPAsyncWebServer configuration page taken from manuals:
// https://randomnerdtutorials.com/esp32-esp8266-input-data-html-form/

#include <esp_camera.h>
#include <WiFi.h>
#include <esp_timer.h>
//#include "img_converters.h"
#include "Arduino.h"

#include "fb_gfx.h"
#include "fd_forward.h"
#include "fr_forward.h"

//#include "driver/rtc_io.h"

// Use "Arduino -> Add a source folder"
// C:\Users\E_CAD\git\ESPAsyncWebServer
// C:\Users\E_CAD\git\AsyncTCP

#include <ESPAsyncWebServer.h>
#include <StringArray.h>

#include <SPIFFS.h>

#include "virtuino_pins.h"
#include "time.h"
#include "Credentials.h"

#include "Bitmap_buffer.h"

// JPEG decoder library
#include <JPEGDecoder.h>

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

#define width_letter 14 //ширина цифр в пикселях  //TODO adjustable from interface
#define number_letter 8 //число цифр в шкале
#define height_letter 26 //высота по y - совпадает с высотой эталона

#define average_count 1 //количество усреднений результатов распознавания
#define average_count_level average_count-3 //число усреднений, которое принимается за положительное при распознавании

#include "sample.h" //образцы эталонов

#define BUILD_IN_LED	4

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { text-align:center; }
    .vert { margin-bottom: 10%; }
    .hori{ margin-bottom: 0%; }
  </style>
</head>
<body>
  <div id="container">
    <h2>ESP32-CAM Last Photo</h2>
    <p>
      <button onclick="capturePhoto()">CAPTURE PHOTO</button>
      <button onclick="location.reload();">REFRESH PAGE</button>
      <button onclick="CAMreboot();">REBOOT</button>
    </p>
  </div>
	<div><img src="jpeg-output01" id="photo" width="60%"></div>
	<div><a href="../jpeg-full-frame"  >Full picture</a></div>
	<div><a href="../params"      >Parameters screen</a></div>
</body>
<script>
  function capturePhoto() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', "/capture", true);
    xhr.send();
  }
  function CAMreboot() {
    var xhr = new XMLHttpRequest();
    xhr.open('GET', "/reboot", true);
    xhr.send();
  }
  function isOdd(n) { return Math.abs(n % 2) == 1; }
</script>
</html>)rawliteral";

const char config_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Saved value to ESP SPIFFS");
      setTimeout(function(){ document.location.reload(false); }, 500);   
    }
  </script></head><body>
  <form action="/get" target="hidden-form">
    Crop area coodinates X1:
	<input type="number" name="inputIntX1" value=%inputIntX1% min="0" max="1600">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    Crop area coodinates Y1:
	<input type="number" name="inputIntY1" value=%inputIntY1% min="0" max="1200">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    Crop area coodinates X2:
	<input type="number" name="inputIntX2" value=%inputIntX2% min="0" max="1600">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    Crop area coodinates Y2:
	<input type="number" name="inputIntY2" value=%inputIntY2% min="0" max="1200">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    Use flashlight (0=OFF; 1=ON):
	<input type="checkbox" name="FlashLED" value="ticked" %FlashLED%>
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    Number of frames to integrate:
	<input type="number" name="V_number_of_sum_frames" value=%V_number_of_sum_frames% min="0" max="20">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
	offset along the Y axis when summing frames and displaying 20 250:
  <input type="number" name="V_offset_y" value=%V_offset_y% min="20" max="250">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
	shifted along the X axis when summing frames and displaying 0 - 50:
  <input type="number" name="V_offset_x" value=%V_offset_x% min="0" max="50">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
	Add. binarization level for searching digits by Y axis (percents):
  <input type="number" name="V_level_find_digital_Y" value=%V_level_find_digital_Y% min="0" max="100">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
    Add. binarization level for searching digits on X axis (percents):
  <input type="number" name="V_level_find_digital_X" value=%V_level_find_digital_X% min="0" max="100">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <form action="/get" target="hidden-form">
	Add. the binarization level when converting to 32 bits:
  <input type="number" name="V_level_convert_to_32" value=%V_level_convert_to_32% min="-100" max="100">
    <input type="submit" value="Submit" onclick="submitMessage()">
  </form><br>
  <iframe style="display:none" name="hidden-form"></iframe>
</body></html>)rawliteral";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
boolean takeNewPhoto = false;
boolean refreshCalcs = false;

HDR frame_buf;
JPEG jpeg_output01;		// Cropped HDR stage

frame area_frame;
frame read_window;			// coordinates of counter values readout window

BitmapBuff *Bitmap = NULL;	// Global pointer to bitmap object

uint16_t pixel_level = 0; //пороговый уровень пикселей на изображении определеный методом Отцу

uint16_t max_letter_x[number_letter]; //массив середины цифры по оси Х

uint32_t l_32[number_letter][300]; //массив после перевода распознаваемых цифр в 32 битное число. Запас по высоте равен высоте экрана// TODO dinamic memory allocation?
uint8_t result[average_count][number_letter]; //накопление результатов распознавания цифр со шкалы


#define max_shift 9*3 //число вариантов сдвига перемещения эталона
int shift_XY[max_shift][2] = { //содержит сдвиг по оси X Y
  {0  ,  0},	//none
  {0  ,  1},   	//up
  {0  ,  2},   	//up
  {0  ,  3},   	//up
  {0  ,  4},   	//up
  {0  , -1},  	//down
  {0  , -2},  	//down
  {0  , -3},  	//down
  {0  , -4},  	//down
  {1  ,  0},   	//right
  {1  ,  1},   	//right up
  {1  ,  2},   	//right up
  {1  ,  3},   	//right up
  {1  ,  4},   	//right up
  {1  , -1},  	//right down
  {1  , -2},  	//right down
  {1  , -3},  	//right down
  {1  , -4},  	//right down
  { -1,  0}, 	//left
  { -1,  1}, 	//left up
  { -1,  2}, 	//left up
  { -1,  3}, 	//left up
  { -1,  4}, 	//left up
  { -1, -1},	//left down
  { -1, -2},	//left down
  { -1, -3},	//left down
  { -1, -4},	//left down
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
//sensor_t * s; //для работы камеры указаитель на структуру сенсора

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

#define size_m3 2048 //размер кольцевого буфера для хранения данных каждую минуту должен быть 256, 512, 1024 ...

//структура для сохранения информации о расчете объема газа
struct Gas_struct {
  uint32_t m3; //значенее объма газа умноженное на 100
  uint32_t minutes; //разница в минутах между предыдущим и текущим измерением
} Gas[size_m3];

uint16_t position_m3 = 0; //позиция сохранения данных

int offset_y_current; //текущее дополнительное смещение по оси Y


void notFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain", "Not found");
}

int genBufferChunk(char *buffer, int maxLen, size_t index, char *DataBuf, size_t DataSize)
{
      size_t max 	 = (ESP.getFreeHeap() / 3) & 0xFFE0;

      // Get the chunk based on the index and maxLen
      size_t len = DataSize - index;
      if (len > maxLen) len = maxLen;
      if (len > max) len = max;
      if (len > 0){
    	  if(0==index){
    		  Serial.printf(PSTR("[WEB] Sending chunked buffer (max chunk size: %4d) "), max);
    	  }
		  memcpy_P(buffer, DataBuf + index, len);
		  Serial.printf(PSTR("."));
      }
      if (len == 0) Serial.printf(PSTR("\r\n"));
      // Return the actual length of the chunk (0 for end of file)
      return len;
}

// Replaces placeholder with stored values
String processor(const String& var){
  //Serial.println(var);
  if(var == "inputIntX1"){
    return String(V[V_CropX1]);
  }
  else if(var == "inputIntX2"){
    return String(V[V_CropX2]);
  }
  else if(var == "inputIntY1"){
    return String(V[V_CropY1]);
  }
  else if(var == "inputIntY2"){
    return String(V[V_CropY2]);
  }
  else if(var == "FlashLED"){
    return String(V[V_Flash]!=0.0?"checked":"");
  }
  else if(var == "V_offset_y"){
    return String(V[V_offset_y]);
  }
  else if(var == "V_offset_x"){
    return String(V[V_offset_x]);
  }
  else if(var == "V_level_find_digital_Y"){
    return String(V[V_level_find_digital_Y]);
  }
  else if(var == "V_level_find_digital_X"){
    return String(V[V_level_find_digital_X]);
  }
  else if(var == "V_level_convert_to_32"){
    return String(V[V_level_convert_to_32]);
  }
  else if(var == "V_number_of_sum_frames"){
    return String(V[V_number_of_sum_frames]);
  }
  return String();
}


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
  Serial.printf("%s", s.c_str());
}
//---------------------------------------------------- printBinary


//---------------------------------------------------- britnes_digital
void find_middle_britnes_digital(HDR *fr_buf, bool show, frame *read_window) {
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

    for (uint16_t y = read_window->Y1; y < read_window->Y2; y++) { //перебор по столбцу в пределах высоты буквы
      for (uint16_t x = x1; x < max_letter_x[dig] + width_letter / 2; x++) { //перебор по строке в пределах ширины одной цифры
        uint32_t i = (y * fr_buf->width + x);
        britnes += fr_buf->buf[i]; //суммируем все значения
      }
    }
    Hemming[dig].britnes_digital = (int)(britnes / (w_l * (read_window->Y2 - read_window->Y1))); //для первой цифры ширина может быть меньше
    if (show)
      Serial.printf("dig=%d, avg. britnes=%d, pixel_level=%d\n", dig, (int)(britnes / (width_letter * (read_window->Y2 - read_window->Y1))), pixel_level);
  }
}
//---------------------------------------------------- britnes_digital


//---------------------------------------------------- find_middle_level_image
uint16_t find_middle_level_image(HDR *fr_buf, bool show) {
  //найти уровень яркости изображения по Отцу

  float av = 0;
  uint32_t f_size = fr_buf->width * fr_buf->height;

  //найти средний уровень пикселей окно табло - засвечено
  // Посчитаем минимальную и максимальную яркость всех пикселей
  for (uint32_t i = 0; i < f_size; i++) {
    av += fr_buf->buf[i];
  }
  av = av / f_size;

  // Гистограмма будет ограничена снизу и сверху значениями min и max,
  // поэтому нет смысла создавать гистограмму размером 256 бинов
  int histSize = fr_buf->max - fr_buf->min + 1;

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
    hist[fr_buf->buf[i] - fr_buf->min]++;

  // Введем два вспомогательных числа:
  int m = 0; // m - сумма высот всех бинов, домноженных на положение их середины
  int n = 0; // n - сумма высот всех бинов
  for (int t = 0; t <= fr_buf->max - fr_buf->min; t++)
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
  for (int t = 0; t < fr_buf->max - fr_buf->min; t++)
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
  threshold += fr_buf->min;

  heap_caps_free(hist); //освободить буфер

  if (show)
    Serial.printf("min =%d max=%d level average=%.0f threshold= %d\n", fr_buf->min, fr_buf->max, av, threshold);
  // Все, порог посчитан, возвращаем его наверх :)
  return (uint16_t)(threshold);
}
//---------------------------------------------------- find_middle_level_image


//---------------------------------------------------- find_digits_y
esp_err_t find_digits_y (HDR *fr_buf, uint16_t mid_level, bool show, frame *read_window) {
  //fr_buf буфер с изображением формата uint16_t
  //mid_level средний уровень яркости изображения
  //add_mid_level повышение уровня для устранения засветки при анализе
  //show вывести информацию на экран

  read_window->Y2 = 0;
  read_window->Y1 = 0;

  float av = 0;
  char buf[50]; //буфер для перевода значений строку и печати на дисплеи

  //поиск среднего уровня
  for (uint8_t y = 0; y < fr_buf->height; y++) { 	// только в пределах экрана по высоте 10-100 строки
    for (uint16_t x = 0; x < fr_buf->width; x++) { 	// ограничим шириной экрана, а не всем изображением F_WIDTH
      uint32_t i = (y * fr_buf->width + x);
      if (fr_buf->buf[i] > mid_level) av++;
    }
  }
  av = (uint16_t) (av / (fr_buf->height));			// average pixel row summary brightness over mid_level level

  for (uint8_t y = 0; y < fr_buf->height; y++) { 	// только в пределах экрана по высоте 10-100 строки
    float av1 = 0;
    for (uint16_t x = 0; x < fr_buf->width; x++) {
      uint32_t i = (y * fr_buf->width + x);
      if (fr_buf->buf[i] > mid_level) av1++;
    }

    if (av1 < av) {			// begin of dark wiew window
    	  read_window->Y1 = y;

          if (show) {
        	  Serial.printf("begin: av=%.0f av1=%.0f Y = %d\n", av, av1, y);
          }
          break;
    }
  }

  for (uint8_t y = fr_buf->height; y > 0 ; y--) { 	// только в пределах экрана по высоте 10-100 строки
    float av1 = 0;
    for (uint16_t x = 0; x < fr_buf->width; x++) {
      uint32_t i = (y * fr_buf->width + x);
      if (fr_buf->buf[i] > mid_level) av1++;
    }

    if (av1 < av) {			// begin of dark wiew window
    	  read_window->Y2 = y;

          if (show) {
        	  Serial.printf("end: av=%.0f av1=%.0f Y = %d\n", av, av1, y);
          }
          break;
    }
  }



  if (show) {
	  Serial.printf("Y_first=%d, Y_last=%d\n", read_window->Y1, read_window->Y2);
  }

//  uint8_t Y_mid = read_window->Y1 + ((read_window->Y2 - read_window->Y1) >> 1);
//  if (read_window->Y2 - read_window->Y1 != height_letter) {
//	  read_window->Y2 = Y_mid + (height_letter >> 1);
//	  read_window->Y1 = Y_mid - (height_letter >> 1);
//  }



  //tft.drawLine(0, Y_first, tft.maxX() - 1, Y_first, COLOR_YELLOW);
  //tft.drawLine(0, Y_last, tft.maxX() - 1, Y_last, COLOR_YELLOW);
  //tft.drawLine(0, Y_mid, tft.maxX() - 1, Y_mid, COLOR_CYAN);



  //вывод на дисплей результатов сохраненных значений
  //tft.setFont(Terminal6x8); //10 pixel
  //tft.fillRectangle (0, info_time - 2, tft.maxX(), info_time + 10, COLOR_BLACK); //очистить часть экрана

  uint8_t pos_1 = (position_m3 - 1) & (size_m3 - 1); //позиция в буфере после места записи -1
  uint8_t pos_2 = (position_m3 - 2) & (size_m3 - 1); //позиция в буфере после места записи -2

  if (Gas[pos_2].minutes != 0) { //если уже сохранено не менее 2-х элементов
    sprintf(buf, "%4d mins %4.2f m3/m", Gas[pos_1].minutes, (Gas[pos_1].m3 - Gas[pos_2].m3) / (Gas[pos_1].minutes * 100.0));
    //tft.drawText(0, info_time, buf, COLOR_WHITE);

    V[V_m3_m] = (Gas[pos_1].m3 - Gas[pos_2].m3) / (Gas[pos_1].minutes * 100.0);
    if (V[V_m3_m] > 1) V[V_m3_m] = 0; //за 1 минуту не может быть больше 1 м3
  }
  else {
    sprintf(buf, "%4d mins %4.2f m3", Gas[pos_1].minutes, Gas[pos_1].m3 / 100.0);
    //tft.drawText(0, info_time, buf, COLOR_WHITE);
    V[V_m3_m] = 0;
  }

  sprintf(buf, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  if (!getLocalTime(&timeinfo)) {
    Serial.printf("Failed to obtain time\n");
    //tft.drawText(tft.maxX() - tft.getTextWidth(buf), info_time, buf, COLOR_RED); //9 * tft.getCharWidth(48)
  }
  else {
    sprintf(buf, "%02d:%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    //tft.drawText(tft.maxX() - tft.getTextWidth(buf), info_time, buf, COLOR_YELLOW);
  }

  uint16_t x_width_min = fr_buf->width; //ширина экрана
  uint16_t x_width_max = 0; //минимальное значенее

  for (uint8_t dig = 0; dig < number_letter; dig++) {
    //поиск максимальной и минимальной ширины определнной цифры
    if (x_width_min > Hemming[dig].x_width) x_width_min = Hemming[dig].x_width;
    if (x_width_max < Hemming[dig].x_width) x_width_max = Hemming[dig].x_width;
  }

/*
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
*/

  if (show) {
	  Serial.printf("Y_first=%d, Y_last=%d\n", read_window->Y1, read_window->Y2);
  }
  return ESP_OK;


}
//---------------------------------------------------- find_digits_y


//---------------------------------------------------- find_digits_x
esp_err_t find_digits_x (HDR *fr_buf, uint16_t mid_level, bool show, frame *read_window) {
  //fr_buf буфер с изображением формата uint16_t
  //mid_level средний уровень яркости изображения
  //add_mid_level повышение уровня для устранения засветки при анализе
  //show вывести информацию на экран

  read_window->X2 = 0;
  read_window->X1 = 0;

  float av = 0;

  //поиск среднего уровня
  for (uint16_t x = 0; x < fr_buf->width; x++) { 	//ограничим шириной экрана, а не всем изображением F_WIDTH
	  for (uint16_t y = read_window->Y1; y < read_window->Y2; y++) { 	//только в пределах экрана по высоте 10-100 строки
		  uint32_t i = (y * fr_buf->width + x);
		  if (fr_buf->buf[i] > mid_level) av++;
	  }
  }
  av = (uint16_t) (av / (fr_buf->width));		// average pixel column summary brightness over mid_level level

  // find left edge
  uint16_t x = 0;
  float av1 = 0.0;
  do{
	  av1 = 0.0;
	  for (uint16_t y = read_window->Y1; y < read_window->Y2; y++) { 	//только в пределах экрана по высоте 10-100 строки
		  uint32_t i = (y * fr_buf->width + x);
		  if (fr_buf->buf[i] > mid_level) av1++;
	  }
	  if (show) Serial.printf("[find_digits_x] Left av=%.0f av1=%.0f X = %d\n", av, av1, x);
	  x++;
  } while (av1 > av);
  read_window->X1 = ++x;

  // find right edge
  x = fr_buf->width;
  do{
	  av1 = 0.0;
	  for (uint16_t y = read_window->Y1; y < read_window->Y2; y++) { 	//только в пределах экрана по высоте 10-100 строки
		  uint32_t i = (y * fr_buf->width + x);
		  if (fr_buf->buf[i] > mid_level) av1++;
	  }
	  if (show) Serial.printf("[find_digits_x] Right av=%.0f av1=%.0f X = %d\n", av, av1, x);
	  x--;
  } while (av1 > av);
  read_window->X2 = --x;


  if (show) {
	  Serial.printf("X_first=%d, X_last=%d\n", read_window->X1, read_window->X2);
  }
  return ESP_OK;


}
//---------------------------------------------------- find_digits_x



//---------------------------------------------------- find_max_digital_X
esp_err_t find_max_digital_X(HDR *fr_buf, uint16_t mid_level, uint8_t treshold,  bool show, frame *read_window) {
  //fr_buf буфер с изображением формата uint16_t
  //mid_level средний уровень изображения
  //add_mid_level повышение следующего уровня для устранения засветки при отображении
  //show вывести информацию на экран

  //поиск границ цифр в найденной строке по X
  uint16_t letter[fr_buf->width]; //массив для поиска максимума по оси Х
  uint16_t letter_min = 0xFFFF;
  uint16_t letter_max = 0;
  uint16_t letter_trs = 0;

  //строим гистограмму подсчет количества единиц по столбцу
  for (uint16_t x = read_window->X1; x < read_window->X2; x++) { //перебор по строке
    letter[x] = 0; //обнуляем начальное значение
    for (uint16_t y = read_window->Y1; y < read_window->Y2 - 2; y++) { //ищем только в пределах обнаруженных цифр
      uint16_t i = (y * fr_buf->width + x);
      if (fr_buf->buf[i] > mid_level) {
        letter[x]++;
      }
    }
    letter_min = letter_min > letter[x] ? letter[x] : letter_min;
    letter_max = letter_max < letter[x] ? letter[x] : letter_max;
    //if(show) Serial.printf("x=%3d letter=%d\n",x,letter[x]);
  }

  letter_trs = letter_min + (letter_max - letter_min)*treshold/100;
  if(show) Serial.printf("[find_max_digital_X] letter_min=%d, letter_trs=%d, letter_max=%d\n",letter_min, letter_trs, letter_max);

  //уточним центры цифр по гистограмме
  uint16_t 	x1 = 0, x2 = 0; //начало и конец цифры
  uint16_t  dig = 0; //номер найденной цифры от 0 до number_letter-1

  for (uint16_t x = read_window->X1; x < read_window->X2; x++) { //перебор по строке
    if (letter[x] > letter_trs) { //if pixel brightness is more than trs limit
      if (x1 != 0) x2 = x; //если было найдено начало установить конец
      else x1 = x; //установить начало
    }
    else { //нет пикселей
      if (x1 != 0 && x2 != 0) { //если были ранее определены пределы показать границы
        if (show) Serial.printf("[find_max_digital_X] x1=%4d x2=%4d x_mid=%4d d_x=%d dig=%d\n", x1, x2, ((x2 - x1) >> 1) + x1, (x2 - x1), dig);

        if (dig > number_letter - 1) { //eсли найдено больше чем цифр в шкале 8 number_letter - 1
          if (show) Serial.printf("[find_max_digital_X] Found more numbers than in the scale along the X axis!!! %d\n", dig);
          return ESP_OK;//ESP_FAIL;
        }
        max_letter_x[dig] = ((x2 - x1) >> 1) + x1;

        Hemming[dig].x_width = (x2 - x1); //сохраним значенее ширины буквы

        if(Hemming[dig].x_width > 10){ // only is character width is plausible //TODO as parameter
        	Hemming[dig].x_width = (Hemming[dig].x_width < 20 ? 20 : Hemming[dig].x_width);		// TODO as parameter
        	dig++;
        }
      }

      //обнулим значение для следующей цифры
      x1 = 0;
      x2 = 0;
    }
  }
  if (dig != number_letter){
    if (show) Serial.printf("Not all scale digits are found on the X axis!!! %d\n", dig);
    return ESP_FAIL;
  }
  return ESP_OK;
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

  if ((y + Y_shift < 240) || (y + Y_shift > 0)) {
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
uint8_t image_recognition(uint8_t dig, uint8_t dig_show, frame *read_window) {
  //dig - какую цифру обрабатываем
  //dig_show какую цифру отображаем если > 8 = не выводим

  //распознавание цифр
  //сравнить с эталоном - расчет расстояния Хемминга
  uint8_t min_dig_number = number_of_samples; //присвоим максимальное значенее

  //вывод цифры со шкалы в двоичном виде
  if (dig == dig_show) {
    Serial.printf("------------------------------\n");
    for (uint8_t y = 0; y < read_window->Y2 - read_window->Y1; y++) { //перебор по Y
      printBinary(l_32[dig][y], "\n");
    }
    Serial.printf("------------------------------\n");
  }

  //вывод цифры со шкалы в HEX формате
  if (V[V_SH_HEX] == 1) {
    Serial.printf("{//%d\n", dig);
    for (uint8_t y = 0; y < read_window->Y2 - read_window->Y1; y++) { //перебор по Y
      Serial.printf("0x%08ux,\t//", l_32[dig][y]);
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
void convert_to_32(HDR *fr_buf, uint16_t mid_level, int16_t add_mid_level, bool show, frame *read_window) {

  /*
    //проверка на максимум уровня
    if (mid_level + add_mid_level > 255)
      add_mid_level = (255 - mid_level);
  */

  for (uint8_t dig = 0; dig < number_letter; dig++) { //всего 8 цифр в шкале последовательно обрабатываем каждую
    for (uint16_t y = read_window->Y1; y < read_window->Y2; y++) { //перебор по столбцу
      l_32[dig][y - read_window->Y1] = 0;
      int x1 = max_letter_x[dig] - width_letter / 2; //для первой цифры размер может быть меньше установленной ширины
      if (x1 < 0) x1 = 0;
      for (uint16_t x = x1; x < max_letter_x[dig] + width_letter / 2; x++) { //перебор по строке в пределах одной цифры
        l_32[dig][y - read_window->Y1] = l_32[dig][y - read_window->Y1] << 1; //сдвиг на 1 позицию
        uint32_t i = (y * fr_buf->width + x);

        if (fr_buf->buf[i] > Hemming[dig].britnes_digital + add_mid_level) { //индивидуальный уровень яркости для каждой цифры
          //        if (fr_buf[i] > mid_level + add_mid_level) { //средний уровень
          if (show || V[V_SH_0_1] != 0) Serial.printf("1");
          l_32[dig][y - read_window->Y1]++;
        }
        else {
          if (show) Serial.printf(" ");
          if (V[V_SH_0_1] == 1) Serial.printf(" ");
          if (V[V_SH_0_1] == 2) Serial.printf("0");
        }
      }
      if (show) Serial.printf("|0x%08ux\n", l_32[dig][y - read_window->Y1]);
      if (V[V_SH_0_1] != 0) Serial.printf("\n");
    }
    if (show) Serial.printf("Letter box middel = %d d_x = %d d_y =%d mid_line_y=%d\n", max_letter_x[dig], width_letter, read_window->Y2 - read_window->Y1, read_window->Y1 + (read_window->Y2 - read_window->Y1) / 2);
    if (V[V_SH_0_1] != 0) Serial.printf("\n");
  }
}
//---------------------------------------------------- convert_to_32


//---------------------------------------------------- dispalay_ttf_B_W
esp_err_t dispalay_ttf_B_W(HDR *fr_buf, int16_t add_mid_level, fb_data_t *output, frame *output_window) {
  //fr_buf буфер с изображением формата uint16_t
  //X0 начальная кордината вывода по оси Х
  //Y0 начальная кордината вывода по оси Y
  //mid_level средний уровень изображения применятся индивидуальный для каждой цифры
  //add_mid_level повышение следующего уровня для устранения засветки при отображении

	uint16_t W = output_window->X2 - output_window->X1;
	uint16_t H = output_window->Y2 - output_window->Y1;

	W = W>fr_buf->width  ? fr_buf->width  : W;
	H = H>fr_buf->height ? fr_buf->height : H;


	for (int y = 0; y < H; y++) //X, Y is a fr_buf coordinates
	for (int x = 0; x < W; x++) {
	  uint8_t dig = 0; //номер цифры табло

	  uint32_t i = (y * fr_buf->width + x); // fr_buf index

	  if (x > max_letter_x[dig] + width_letter / 2) { //если текущее значенее в пределах цифры, то использовать соответствующее значенее яркости
		  dig++; //перейти к следующей цифре
		  //Serial.printf("[dispalay_ttf_B_W] Next dig=%d, britnes_digital=%d\r\n", dig, Hemming[dig].britnes_digital);
		  if (dig > number_letter) dig = number_letter - 1; //если больше цифр то принимать яркость последней
	  }

	  uint32_t colour=FACE_COLOR_BLUE;
	  if (fr_buf->buf[i] >= Hemming[dig].britnes_digital + add_mid_level){ //индивидуальный уровень яркости для каждой цифры
		  colour = FACE_COLOR_WHITE;
	  }
	  fb_gfx_drawFastHLine(output, x + output_window->X1, y + output_window->Y1, 1, colour);
	}
  return ESP_OK;
}
//---------------------------------------------------- dispalay_ttf_B_W

/*
//---------------------------------------------------- HDR_2_jpeg
esp_err_t HDR_2_jpeg(HDR *fr_buf, bool show, JPEG *jpeg_Out) {
  uint32_t tstart;
  tstart = clock();
  if (show) {
	  Serial.printf(PSTR("[HDR_2_jpeg] Free heap before call: %d bytes\n"), ESP.getFreeHeap());
  }
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fr_buf->width, fr_buf->height, 3);
  if (!image_matrix) {
      Serial.println("[HDR_2_jpeg] dl_matrix3du_alloc failed");
  }
  else{
	  // Rescale HDR to 8bit bitmap matrix
	  for (uint16_t y = 0; y < fr_buf->height; y++)
	  for (uint16_t x = 0; x < fr_buf->width; x++) {
		  uint32_t i = (y * fr_buf->width + x); // fr_buf->buf adress from coordinates
		  // rescale
		  uint32_t t = fr_buf->buf[i]-fr_buf->min;
		  t *= 256;
		  t /= (fr_buf->max - fr_buf->min)+1;
		  image_matrix->item[i*3+0] = (uint8_t) t;
		  image_matrix->item[i*3+1] = (uint8_t) t;
		  image_matrix->item[i*3+2] = (uint8_t) t;
	  }
      size_t out_len = image_matrix->w * image_matrix->h * 3;
      free(jpeg_Out->buf);
      jpeg_Out->buf = NULL;
	  if(!fmt2jpg(image_matrix->item, out_len, image_matrix->w, image_matrix->h, PIXFORMAT_RGB888, 80, &jpeg_Out->buf, &jpeg_Out->buf_len)){
		  Serial.println("[HDR_2_jpeg] JPEG compression failed");
	  }
	  else{
		  Serial.printf("[HDR_2_jpeg] JPEG compression done. Jpeg size is %i bytes\r\n", jpeg_Out->buf_len);
	  }
  }
  dl_matrix3du_free(image_matrix);
  image_matrix = NULL;
  if (show) {
	  Serial.printf(PSTR("[HDR_2_jpeg] Free heap after call: %d bytes\n"), ESP.getFreeHeap());
	  Serial.printf(PSTR("[HDR_2_jpeg] time consume: %u ms\n"), clock() - tstart);
  }
  return ESP_OK;
}
//---------------------------------------------------- HDR_2_jpeg
*/


//---------------------------------------------------- sum_frames
esp_err_t sum_frames(HDR *fr_buf, bool show, frame *area_frame, uint8_t count) {
  uint32_t tstart;
  fb = NULL;

  uint16_t W = area_frame->X2 - area_frame->X1;
  uint16_t H = area_frame->Y2 - area_frame->Y1;

  Serial.printf(PSTR("[sum_frames] Free heap: %d bytes, Free PSRAM: %d bytes\n"), ESP.getFreeHeap(), ESP.getFreePsram());

  //накопление кадров - проинтегрировать несколько кадров для устранения шумов
  tstart = clock();

  uint8_t frame_c = count;
  sensor_t * s = esp_camera_sensor_get();

  // Clear HDR buffer
  for(uint32_t i = 0; i < frame_buf.buf_len; i++){
	  frame_buf.buf[i] = 0x0000;
  }
  fr_buf->max 	 = 0x0000;
  fr_buf->min 	 = 0xFFFF;

  // Prepare camera for image capture quality parameters
  s->set_framesize(s, FRAMESIZE_UXGA);
  s->set_pixformat(s, PIXFORMAT_JPEG);
  s->set_quality(s, 5);

  for (uint8_t frames = 0; frames < frame_c; frames++) { //усредненее по кадрам frame_count // +1 for last low quality photo for visualization in browser
    if (fb) { //освободить буфер
      esp_camera_fb_return(fb);
      fb = NULL;
    }

    Serial.printf("[sum_frames] Camera capture: ");

    if(V[V_Flash] != 0.0) {
    	digitalWrite(BUILD_IN_LED, HIGH);
    	delay(5);
    };

    fb = esp_camera_fb_get(); //получить данные от камеры

    digitalWrite(BUILD_IN_LED, LOW);

    if (!fb) {
      Serial.printf("Camera capture failed\n");
      return ESP_FAIL;
    }
    else{
      Serial.printf("Camera capture OK\n");
    }

	Serial.printf("[sum_frames] Framebuffer from camera fb size is %u bytes\r\n", fb->len);

	// Huge JPEG crop and store cropped area to HDR buffer
	if (fb->format == PIXFORMAT_JPEG){

		boolean decoded = JpegDec.decodeArray(fb->buf, fb->len);
		if (decoded) {
			// print information about the image to the serial port
			//jpegInfo();

			// render the image onto the HDR container at given coordinates
			jpegRender(fr_buf, area_frame);
		}
		else {
			Serial.println("[sum_frames] Jpeg file format not supported!");
		}

	}
	else{


		uint32_t i_max = fb->height * fb->width; //максимальное значенее массива для данного экрана

		fr_buf->min = 0xFFFF; // reset min before new frame capture

		//proceed only cropped area of the frame
		for (uint16_t y = area_frame->Y1; y < area_frame->Y2; y++)
		for (uint16_t x = area_frame->X1; x < area_frame->X2; x++) {

			  uint32_t i = (y * fb->width + x); // fb->buf adress from coordinates
			  uint32_t j = (y - area_frame->Y1) * W + (x-area_frame->X1); // fr_buf adress from coordinates

			  if (fb->format == PIXFORMAT_GRAYSCALE) {
				  fr_buf->buf[j] += fb->buf[i]; // accumulate cropped area
			  }
			  else { //если RGB565 берем 8 битный буфер и преобразуем в 16 битный
				  i <<= 1; //если RGB565
				  //https://github.com/techtoys/SSD2805/blob/master/Microchip/Include/Graphics/gfxcolors.h
				  fr_buf->buf[j] += (uint16_t)(fb->buf[i]) << 8 | (uint16_t)(fb->buf[i + 1]); //преобразуем в 16 битное
			  }
			  fr_buf->max = fr_buf->max > fr_buf->buf[j] ? fr_buf->max : fr_buf->buf[j];
			  fr_buf->min = fr_buf->min < fr_buf->buf[j] ? fr_buf->min : fr_buf->buf[j];
		 }
	}

  } //суммирование по кадрам

  Serial.printf(PSTR("[sum_frames] Free heap: %d bytes, Free PSRAM: %d bytes\n"), ESP.getFreeHeap(), ESP.getFreePsram());


  // camera shoot for preview in browser
	if (fb) { //освободить буфер
		esp_camera_fb_return(fb);
		fb = NULL;
	}

	s->set_framesize(s, FRAMESIZE_QVGA);
	s->set_pixformat(s, PIXFORMAT_JPEG);	// YUV format gives black picture :( RGB888 works bad with fmt2rgb888 ((

    Serial.printf("\r\n[sum_frames] Preview camera capture: ");
    if(V[V_Flash] != 0.0) {
    	digitalWrite(BUILD_IN_LED, HIGH);
    	delay(5);
    };
    fb = esp_camera_fb_get(); //получить данные от камеры
    digitalWrite(BUILD_IN_LED, LOW);
    if (!fb) {
      Serial.printf("failed\n");
      return ESP_FAIL;
    }
    else{
      Serial.printf("OK\n");
    }
	Serial.printf("[sum_frames] Framebuffer with preview image from camera fb size is %u bytes\r\n", fb->len);

	dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3); // Allocate RGB888 memory area for drawing
	if (!image_matrix) {
		Serial.println("[sum_frames] dl_matrix3du_alloc failed");
	}
	else{
		if(!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)){	// Fill allocated area with camera picture
			Serial.println("[sum_frames] convert preview frame to rgb888 failed");
		}
		else{

		    fb_data_t colour_buf;								// pointer to picture for drawing procedures
		    colour_buf.width = fb->width;
		    colour_buf.height = fb->height;
		    colour_buf.bytes_per_pixel = 3;
		    colour_buf.format = FB_RGB888;
			colour_buf.data = image_matrix->item;

			// draw rectangle box
			int x = (int)(area_frame->X1)						*320;	// rescale coordinates to smaller resolution
			int y = (int)(area_frame->Y1)						*240;
			int w = (int)(area_frame->X2 - area_frame->X1 + 1)	*320;
			int h = (int)(area_frame->Y2 - area_frame->Y1 + 1)	*240;

			x /= 1600;
			y /= 1200;
			w /= 1600;
			h /= 1200;

			fb_gfx_drawFastHLine(&colour_buf, x, y, w, 		FACE_COLOR_GREEN);
			fb_gfx_drawFastHLine(&colour_buf, x, y+h-1, w, 	FACE_COLOR_GREEN);
			fb_gfx_drawFastVLine(&colour_buf, x, y, h, 		FACE_COLOR_GREEN);
			fb_gfx_drawFastVLine(&colour_buf, x+w-1, y, h, 	FACE_COLOR_GREEN);


			if (fb) { //освободить буфер
				esp_camera_fb_return(fb);
				fb = NULL;
			}


			dl_matrix3du_free(image_matrix);
			image_matrix = NULL;
		}

	}

	// Prepare camera for full size preview with high compression quality parameters
	s->set_framesize(s, FRAMESIZE_UXGA);
	s->set_pixformat(s, PIXFORMAT_JPEG);
	s->set_quality(s, 50);
    Serial.printf("\r\n[sum_frames] Preview camera capture: ");
    if(V[V_Flash] != 0.0) {
    	digitalWrite(BUILD_IN_LED, HIGH);
    	delay(5);
    };
    fb = esp_camera_fb_get(); //получить данные от камеры
    digitalWrite(BUILD_IN_LED, LOW);
    if (!fb) {
      Serial.printf("failed\n");
      return ESP_FAIL;
    }
    else{
      Serial.printf("OK\n");
    }
	Serial.printf("[sum_frames] Framebuffer with full size hi-compressed preview from camera fb size is %u bytes\r\n", fb->len);

	Serial.printf(PSTR("[sum_frames] Free heap: %d bytes, Free PSRAM: %d bytes\n"), ESP.getFreeHeap(), ESP.getFreePsram());

	if (show) {
		Serial.printf("[sum_frames] Summary capture time for all frames: %lu ms of %d frames\n", clock() - tstart, frame_c);
	}
	return ESP_OK;
}
//---------------------------------------------------- sum_frames

//---------------------------------------------------- setup
void setup() {
													// Init serial port for debug output
  Serial.begin(115200);
  Serial.setDebugOutput(true);

  	  	  	  	  	  	  	  	  	  	  	  	  	// Camera configuration
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

  if(!psramFound()){
    Serial.printf(PSTR("[SETUP] psram not found. Change board to PSRAM enabled\n"));
    delay(10000);
    ESP.restart();
  }

  config.fb_count = 1;

  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; //PIXFORMAT_GRAYSCALE; //PIXFORMAT_RGB565;
  config.jpeg_quality = 5;				// Lower values - less compression - bigger size

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    delay(2000);
    ESP.restart();
  }
  else {
	  Serial.println("Camera init OK");
	  fb = esp_camera_fb_get(); // take first shoot to initialize camera white balance
  }

  // Camera initialized and first picture taken to get a correct white balance

  WiFi_Connect();




  // Init web server

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/html", index_html);
  });

  server.on("/capture", HTTP_GET, [](AsyncWebServerRequest * request) {
    takeNewPhoto = true;
    request->send_P(200, "text/plain", "Taking Photo");
  });

  server.on("/jpeg-full-frame", HTTP_GET, [](AsyncWebServerRequest * request) {

      // Chunked response, we calculate the chunks based on free heap (in multiples of 32)
      // This is necessary when a TLS connection is open since it sucks too much memory
	  // https://github.com/helderpe/espurna/blob/76ad9cde5a740822da9fe6e3f369629fa4b59ebc/code/espurna/web.ino - Thanks A LOT!
	  if(!fb){
		  request->send(200, "text/text", "No picture");
	  }
	  else{
		  Serial.printf(PSTR("[MAIN] Free heap: %d bytes\n"), ESP.getFreeHeap());
		  AsyncWebServerResponse *response = request->beginChunkedResponse("image/jpeg",[](uint8_t *buffer, size_t maxLen, size_t index) -> size_t{
			  return genBufferChunk((char *)buffer, (int)maxLen, index, (char *)fb->buf, fb->len);
		  });
		  response->addHeader("Content-Disposition", "inline; filename=capture.jpeg");
		  request->send(response);
	  }
  });

  server.on("/jpeg-output01", HTTP_GET, [](AsyncWebServerRequest * request) {

      // Chunked response, we calculate the chunks based on free heap (in multiples of 32)
      // This is necessary when a TLS connection is open since it sucks too much memory
	  // https://github.com/helderpe/espurna/blob/76ad9cde5a740822da9fe6e3f369629fa4b59ebc/code/espurna/web.ino - Thanks A LOT!

	  if(jpeg_output01.buf == NULL){
		  request->send(200, "text/text", "No picture");
	  }
	  else{
	  	  Serial.printf(PSTR("[MAIN] Free heap: %d bytes\n"), ESP.getFreeHeap());
		  AsyncWebServerResponse *response = request->beginChunkedResponse("image/jpeg",[](uint8_t *buffer, size_t maxLen, size_t index) -> size_t{
			  return genBufferChunk((char *)buffer, (int)maxLen, index, (char *)jpeg_output01.buf, jpeg_output01.buf_len);
		  });
		  response->addHeader("Content-Disposition", "inline; filename=capture.jpeg");
		  request->send(response);
	  }
  });


  server.on("/reboot", HTTP_GET, [](AsyncWebServerRequest * request) {
     request->send_P(200, "text/plain", "Rebooting");
     ESP.restart();
  });

  server.on("/params", HTTP_GET, [](AsyncWebServerRequest *request){
	    request->send_P(200, "text/html", config_html, processor);
  });

  // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
  server.on("/get", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    // GET inputInt value on <ESP_IP>/get?inputInt=<inputMessage>

    if 		(request->hasParam("inputIntX1")) {
		inputMessage = request->getParam("inputIntX1")->value();
		V[V_CropX1] = inputMessage.toInt();

		store_check_limits(V[V_CropX1], 0, V[V_CropX2], "/V_CropX1.txt");

		delay(100);
		ESP.restart();
    }
    else if (request->hasParam("inputIntX2")) {
        inputMessage = request->getParam("inputIntX2")->value();
        V[V_CropX2] = inputMessage.toInt();

        store_check_limits(V[V_CropX2], V[V_CropX1], 1600, "/V_CropX2.txt");

		delay(100);
		ESP.restart();
    }
    else if (request->hasParam("inputIntY1")) {
        inputMessage = request->getParam("inputIntY1")->value();
        V[V_CropY1] = inputMessage.toInt();

        store_check_limits(V[V_CropY1], 0, V[V_CropY2], "/V_CropY1.txt");

		delay(100);
		ESP.restart();
    }
    else if (request->hasParam("inputIntY2")) {
        inputMessage = request->getParam("inputIntY2")->value();
        V[V_CropY2] = inputMessage.toInt();

        store_check_limits(V[V_CropY2], V[V_CropY1], 1200, "/V_CropY2.txt");

		delay(100);
		ESP.restart();
    }
    else if (request->hasArg("FlashLED")) {
        inputMessage = request->arg("FlashLED");

        Serial.printf("FlashLED inputMessage = %s\r\n", inputMessage.c_str());

        V[V_Flash] = 1.0;

        store_check_limits(V[V_Flash], 0, 1, "/V_Flash.txt");
    }
    else if (request->hasParam("V_offset_y")) {
        inputMessage = request->getParam("V_offset_y")->value();
        V[V_offset_y] = inputMessage.toInt();

        store_check_limits(V[V_offset_y], 20, 250, "/V_offset_y.txt");

        refreshCalcs = true;
    }
    else if (request->hasParam("V_offset_x")) {
        inputMessage = request->getParam("V_offset_x")->value();
        V[V_offset_x] = inputMessage.toInt();

        store_check_limits(V[V_offset_x], 0,   50, "/V_offset_x.txt");

        refreshCalcs = true;
    }
    else if (request->hasParam("V_level_find_digital_Y")) {
        inputMessage = request->getParam("V_level_find_digital_Y")->value();
        V[V_level_find_digital_Y] = inputMessage.toInt();

        store_check_limits(V[V_level_find_digital_Y], 0, 100, "/V_level_find_digital_Y.txt");

        refreshCalcs = true;
    }
    else if (request->hasParam("V_level_find_digital_X")) {
        inputMessage = request->getParam("V_level_find_digital_X")->value();
        V[V_level_find_digital_X] = inputMessage.toInt();

        store_check_limits(V[V_level_find_digital_X], 0, 100, "/V_level_find_digital_X.txt");

        refreshCalcs = true;
    }
    else if (request->hasParam("V_level_convert_to_32")) {
        inputMessage = request->getParam("V_level_convert_to_32")->value();
        V[V_level_convert_to_32] = inputMessage.toInt();

        store_check_limits(V[V_level_convert_to_32],  -100, 100, "/V_level_convert_to_32.txt");

        refreshCalcs = true;
    }
    else if (request->hasParam("V_number_of_sum_frames")) {
        inputMessage = request->getParam("V_number_of_sum_frames")->value();
        V[V_number_of_sum_frames] = inputMessage.toInt();

        store_check_limits(V[V_number_of_sum_frames],  1, 20, "/V_number_of_sum_frames.txt");
    }
    else {
    	inputMessage = "No message sent";

        V[V_Flash] = 0.0;
        store_check_limits(V[V_Flash], 0, 1, "/V_Flash.txt");
    }
    Serial.println(inputMessage);
    request->send(200, "text/text", inputMessage);
  });

  server.onNotFound(notFound);


  //init and get the local time

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Set timezone to Ukraine EET
  setenv("TZ", "EET-2EEST,M3.5.0/3,M10.5.0/4", 1);

  if (!getLocalTime(&timeinfo)) { //получим время начала записи сохранения м3
    Serial.printf("Failed to obtain time\n");
    return;
  }

  // Initialize SPIFFS and restore parameters
  if(!SPIFFS.begin(true)){
    Serial.println("An Error has occurred while mounting SPIFFS");
  }

  init_V(); //инициализация начальных данных from SPIFFS
  
  for(uint8_t dig = 0; dig < number_letter; dig++) //обнулить частоту использования эталонов
    for(uint8_t i = 0; i < number_of_samples; i++) 
      used_samples[i][dig] = 0;

  // prepare flashlight
  pinMode(BUILD_IN_LED, OUTPUT);

  // Start server
  server.begin();

  // Prepare and allocate HDR buffer for cropped picture area
  area_frame.X1 = (int)V[V_CropX1];
  area_frame.Y1 = (int)V[V_CropY1];
  area_frame.X2 = (int)V[V_CropX2];
  area_frame.Y2 = (int)V[V_CropY2];

  frame_buf.width 	= area_frame.X2 - area_frame.X1;	  //fr_buf size need to be defined for cropping algorithm
  frame_buf.height 	= area_frame.Y2 - area_frame.Y1;
  frame_buf.buf_len = frame_buf.width * frame_buf.height;

  uint32_t f8  = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  frame_buf.buf = (uint16_t *)heap_caps_calloc(1, frame_buf.buf_len * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (frame_buf.buf == NULL) {
    Serial.printf("malloc failed for HDR frame_buf\n");
    delay(10000);
    ESP.restart();
  }
  else {
    Serial.printf("malloc succeeded for HDR frame_buf. Taken 8BIT chunks = %d\n", f8 - heap_caps_get_free_size(MALLOC_CAP_8BIT));
  }



  // init internal memory and variables

  for (uint8_t dig = 0; dig < number_letter; dig++) {
    Hemming[dig].dig_defined = 10; //заносим первоначально максимальное число вне диапазона 0-9
  }

  for (uint16_t i = 0; i < size_m3; i++) { //обнулим буфер сохранения значений
    Gas[i].m3 = 0;
    Gas[i].minutes = 0;
  }
  Gas[0].minutes = 1; //подсчет времени сначала для текущего элемента

  //Gas_minute_Ticker.attach(60, m3_calculate); //вызывать расчета объма газа каждую минуту 60


  // Take first photo immediately
  takeNewPhoto = true;
}
//---------------------------------------------------- setup


//---------------------------------------------------- WiFi_Connect
void WiFi_Connect()
{
  // Wi-Fi connection
  uint32_t timeout = millis();

  WiFi.begin(ssid, password);
  WiFi.setHostname("counercam");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.printf(".");
    if (millis() - timeout > 5000) break;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi connected.\nCamera Stream Ready! Go to: http://");
    Serial.printf("%s\n", WiFi.localIP().toString().c_str());
    Serial.printf("Hostname: %s\n", WiFi.getHostname());
  }
  else { //create AP
    WiFi.softAP("ESP32", "87654321");
    Serial.printf("\nWiFi %s not found create AP Name - 'ESP32' Password - '87654321'\n", ssid);
    Serial.printf("Camera Stream Ready! Go to: http://");
    Serial.printf("%s\n", WiFi.softAPIP().toString().c_str());
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

  //tft.setGFXFont(&FreeSansBold24pt7b); // Set current font
  //tft.getGFXTextExtent("0", 0, info_result, &w, &h); // Get string extents
  h += info_result;
  //  Serial.printf("info_result=%d w=%d h=%d\n",info_result,w,h);

  //  tft.setFont(Trebuchet_MS16x21); //22 pixel for size 3 Trebuchet_MS16x21

  //tft.fillRectangle (0, info_result - 2, tft.maxX(), h + 7, COLOR_BLACK); //очистить часть экрана GFXFont привязан верхней точкой

  //найти максимальную частоту вхождения цифр после опознавания
  for (uint8_t dig = 0; dig < number_letter; dig++) { //number_letter
    defined = find_max_number(dig);
    sprintf(buf, "%d%c", defined,'\0');
    //обновленее первоначально предопределенного набора символов если частота определения символа более 7 и рассояние Хемминга менее Hemming_level
    if ((frequency[defined][dig] > average_count_level) && (Hemming[dig].min_Hemming < Hemming_level)) {
      if (defined != Hemming[dig].dig_defined) { //корректно обнаружили первый раз
        //        Serial.printf("Change defined digital in position=%d from=%d to %d\n", dig, Hemming[dig].dig_defined, defined);
        Hemming[dig].dig_defined = defined;
        //tft.drawGFXText(next_x, h, buf, COLOR_YELLOW); // Print string
        //        tft.drawText(next_x, info_result,buf,COLOR_YELLOW); //цифра опознана с большой вероятностью
      }
      //else tft.drawGFXText(next_x, h, buf, COLOR_GREEN); //цифра распознана корректно уже неоднократно
    }
    //else tft.drawGFXText(next_x, h, buf, COLOR_RED);

    T_0 += defined;
    next_x += w; //шаг между цифрами

    if (dig == 4) {
      T_0 += ".";
      //tft.drawGFXText(next_x, h, ".", COLOR_GREEN);
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

  //tft.setFont(Terminal6x8); //10 pixel
  //tft.fillRectangle (0, info_Hemming - 2, tft.maxX(), info_Hemming + 24, COLOR_BLACK); //очистить часть экрана для расстояния Хеминга и частоты

  for (uint8_t dig = 0; dig < number_letter; dig++) {
    sprintf(buf, "|%3d %c", Hemming[dig].min_Hemming,'\0');
    //next_x = max(tft.getTextWidth(T_1), tft.getTextWidth(T_2));
    //if (next_x != 0) next_x -= tft.getTextWidth(" ");

    //if (Hemming[dig].min_Hemming < Hemming_level)
      //tft.drawText(next_x, info_Hemming, buf, COLOR_GREEN); //печатать каждую цифру со смещенеем на экране дисплея
    //else
      //tft.drawText(next_x, info_Hemming, buf, COLOR_RED); //печатать каждую цифру со смещенеем на экране дисплея

    //    Serial.printf("%3d %3d '%s'\n",next_x,tft.getTextWidth(T_1),T_1.c_str());
    T_1 += buf;

    sprintf(buf, "|%3d %c", Hemming[dig].frequency,'\0');
    //if (Hemming[dig].frequency > average_count_level)
      //tft.drawText(next_x, info_frequency, buf, COLOR_GREEN); //печатать каждую цифру со смещенеем на экране дисплея
    //else
      //tft.drawText(next_x, info_frequency, buf, COLOR_RED); //печатать каждую цифру со смещенеем на экране дисплея

    //    Serial.printf("%3d %3d %3d '%s'\n",next_x,tft.getTextWidth(T_2),tft.getTextWidth(T_2)-5,T_2.c_str());
    T_2 += buf;
  }
  T_1 += "|";
  T_2 += "|";
  //  Serial.printf("T_1 = %s\n",T_2.c_str());
}
//---------------------------------------------------- show_result


//---------------------------------------------------- loop
void loop() {

	#define min_max_offset_y_test 3 //значенее смещения +/-1 или 0
	uint16_t Sum_min_Hemming[min_max_offset_y_test]; //количество вариантов поиска смещения по оси Y для автоматической подстройки
	static uint8_t WiFi_Lost = 0; //счетчик потери связи WiFi


	static BitmapBuff BitmapObj(frame_buf.width, frame_buf.height*3);		// bitmap buffer for intermediate results output

	Bitmap = &BitmapObj;

	if (takeNewPhoto) {

	  takeNewPhoto = false;


	  // analyze camera view and get the area of interest coordinates
	  // [get_edged_shoot]
	  {

			fb = NULL;
			sensor_t * s = esp_camera_sensor_get();

			Serial.printf(PSTR("[get_edged_shoot] Free heap: %d bytes, Free PSRAM: %d bytes\n"), ESP.getFreeHeap(), ESP.getFreePsram());

			// Prepare camera for image capture quality parameters
			s->set_framesize(s, FRAMESIZE_QVGA);
			s->set_pixformat(s, PIXFORMAT_GRAYSCALE);
			//s->set_quality(s, 5);


			if (fb) { //освободить буфер
				esp_camera_fb_return(fb);
				fb = NULL;
			}

			Serial.printf("[get_edged_shoot] Camera capture: ");

			if(V[V_Flash] != 0.0) {
				digitalWrite(BUILD_IN_LED, HIGH);
				delay(5);
			};

			fb = esp_camera_fb_get(); //получить данные от камеры

			digitalWrite(BUILD_IN_LED, LOW);

			if (!fb) {
				Serial.printf("Camera capture failed\n");
				return ESP_FAIL;
			}
				else{
				Serial.printf("Camera capture OK\n");
			}


			// following code is based on
			// https://rranddblog.wordpress.com/canny-edge-detection-c-optimisation-1/
			// https://github.com/rrandd/Canny-Edge-Detection/tree/master/C%2B%2B/Canny%20Edge%20Detection
			// https://habr.com/ru/post/114589/

			// apply Gaussian blur to fb image buffer
			dl_matrix3du_t *imgblur 	= dl_matrix3du_alloc(1, fb->width, fb->height, 1);		// Gaussian blurred image and result image
			dl_matrix3du_t *imggraddir 	= dl_matrix3du_alloc(1, fb->width, fb->height, 1);		// Image with gradients directions
			dl_matrix3du_t *imggrad 	= dl_matrix3du_alloc(1, fb->width, fb->height, 1);		// Image with gradients magnitudes
			dl_matrix3du_t *imgnm 		= imgblur;													// Result image will re-use blurred image as a storage memory

			if (!imgblur || !imggraddir || !imggrad) {
				Serial.println("[get_edged_shoot] dl_matrix3du_alloc failed");
			}
			else{

				// Definitions
				uint32_t blurpixel=0;
				uint32_t pixelweight = 0;
				uint32_t i = 0;
				uint32_t j = 0;

				//---------------------------------------------------------------------------------------------
				// --------------------- Gaussian blur -----------------------

				// Define gaussian blur weightings array
				int weighting[5][5] =
				{
				{ 2, 4, 5, 4, 2},
				{ 4, 9,12, 9, 4},
				{ 5,12,15,12, 5},
				{ 4, 9,12, 9, 4},
				{ 2, 4, 5, 4, 2}
				};

				// Get each pixel and apply the Gaussian blur filter
				for (int x = 2; x <= fb->width - 2; x++) {
					for (int y = 2; y <= fb->height - 2; y++) {

						// Clear blurpixel
						blurpixel = 0;

						// +- 2 for each pixel and calculate the weighting
						for (int dx = -2; dx <= 2; dx++) {
							for (int dy = -2; dy <= 2; dy++) {
								pixelweight = weighting[dx+2][dy+2];

								i = ((y+dy) * fb->width + (x+dx)); // fb->buf adress from coordinates

								// Apply weighting
								blurpixel = blurpixel + fb->buf[i] * pixelweight;
							}
						}
						// Write pixel to blur image
						j = y * imgblur->w + x; // imgblur adress from coordinates
						imgblur->item[j] = (int)(blurpixel / 159);

					}
				}

				//---------------------------------------------------------------------------------------------
				// --------------------- Gradient intensity pass -----------------------
				{
					int pix[3] = { 0,0,0 };
					int gradx = 0, grady = 0;
					int graddir = 0, grad = 0;
					double tempa = 0, temps = 0, tempr = 0;
					uint32_t i = 0;

					// Get pixels and calculate gradient and direction
					for (int x = 1; x <= fb->width-1; x++) {
						for (int y = 1; y <= fb->height-1; y++) {

							// Get source pixels to calculate the intensity and direction
							i = y * imgblur->w + (x - 1); 	// imgblur adress from coordinates
							pix[1] = imgblur->item[i]; 		// pixel left

							i = (y - 1) * imgblur->w + x; 	// imgblur adress from coordinates
							pix[2] = imgblur->item[i]; 		// pixel above

							i = y * imgblur->w + x; 		// imgblur adress from coordinates
							pix[0] = imgblur->item[i]; 		// main pixel

							// get value for x gradient
							gradx = pix[0] - pix[1];

							// get value for y gradient
							grady = pix[0] - pix[2];

							// Calculate gradient direction
							// We want this rounded to 0,1,2,3 which represents 0, 45, 90, 135 degrees
							//graddir = (int)(abs(atan2(grady, gradx)) + 0.22) * 80;
							// atan2 approximation
							if (max(abs(gradx), abs(grady)) == 0) {
								tempa = 0;
							}
							else {
								tempa = min(abs(gradx), abs(grady)) / max(abs(gradx), abs(grady));
							}
							temps = tempa * tempa;

							tempr = ((-0.0464964749 * temps + 0.15931422) * temps - 0.327622764) * temps * tempa + tempa;

							// Now sort out quadrant
							if (abs(grady) > abs(gradx)) tempr = 1.57079637 - tempr;
							if (gradx < 0) tempr = 3.14159274 - tempr;
							if (grady < 0) tempr = -tempr;
							graddir = (int)(abs(tempr) + 0.22) * 80;
							imggraddir->item[i] = graddir;

							// Calculate gradient
							// grad = (int)sqrt(gradx * gradx + grady * grady) * 2;
							// imggrad->item[i] = grad;

								// Get absolute values for both gradients
								gradx = abs(gradx);
								grady = abs(grady);
								// Calculate gradient length (hypotenuse=(short side * 0.414) + long side)
								if (gradx > grady) {
									grad = (grady * 414) / 1000 + gradx;
								}
								else {
									grad = (gradx * 414) / 1000 + grady;
								}
							imggrad->item[i] = grad * 2;



						}
					}
				}



				//---------------------------------------------------------------------------------------------
				// --------------------- Non-Maximal Suppression -----------------------
				// Definitions
				int sensitivity = 10;
				int graddir = 0, grad = 0;

				// Get each pixel and apply the blur filter
				for (int x = 2; x <= fb->width - 2; x++) {
					for (int y = 2; y <= fb->height - 2; y++) {

						i  = y * fb->width + x; 		// imggrad adress from coordinates

						// First check that current pixel's gradient above the threshold
						if (imggrad->item[i] >= sensitivity) {

							// Get gradient direction
							graddir = imggraddir->item[i];		// Remember this was multiplied by 80 for the sake of display

							// If angle = 0
							if (graddir == 0) {
								// Is pixel local maximal

								int j1 = (y-1) * fb->width + x; 		// imggrad adress from coordinates
								int j2 = (y+1) * fb->width + x; 		// imggrad adress from coordinates

								if (imggrad->item[i] >= imggrad->item[j1] && imggrad->item[i] >= imggrad->item[j2]) {
									// Write pixel to as max
									imgnm->item[i] = 255;
									// Supress other two
									imgnm->item[j1] = 0;
									imgnm->item[j2] = 0;

								}
								else {
									// Supress pixel
									imgnm->item[i] = 0;
								}
							}

							// If angle = 45 degrees
							else if (graddir == 80) {
								// Is pixel local maximal

								int j1 = (y-1) * fb->width + (x+1); 		// imggrad adress from coordinates
								int j2 = (y+1) * fb->width + (x-1); 		// imggrad adress from coordinates
								if (imggrad->item[i] >= imggrad->item[j1] && imggrad->item[i] >= imggrad->item[j2]) {
									// Write pixel to as max
									imgnm->item[i] = 255;
									// Supress other two
									imgnm->item[j1] = 0;
									imgnm->item[j2] = 0;

								}
								else {
									// Supress pixel
									imgnm->item[i] = 0;
								}
							}

							// If angle = 90 degrees
							else if (graddir == 160) {
								// Is pixel local maximal

								int j1 = y * fb->width + (x-1); 		// imggrad adress from coordinates
								int j2 = y * fb->width + (x+1); 		// imggrad adress from coordinates
								if (imggrad->item[i] >= imggrad->item[j1] && imggrad->item[i] >= imggrad->item[j2]) {
									// Write pixel to as max
									imgnm->item[i] = 255;
									// Supress other two
									imgnm->item[j1] = 0;
									imgnm->item[j2] = 0;

								}
								else {
									// Supress pixel
									imgnm->item[i] = 0;
								}
							}

							// If angle = 135 degrees
							else if (graddir == 240) {
								// Is pixel local maximal
								int j1 = (y-1) * fb->width + (x-1); 		// imggrad adress from coordinates
								int j2 = (y+1) * fb->width + (x+1); 		// imggrad adress from coordinates
								if (imggrad->item[i] >= imggrad->item[j1] && imggrad->item[i] >= imggrad->item[j2]) {
									// Write pixel to as max
									imgnm->item[i] = 255;
									// Supress other two
									imgnm->item[j1] = 0;
									imgnm->item[j2] = 0;

								}
								else {
									// Supress pixel
									imgnm->item[i] = 0;
								}
							}

						}
						else {

							// Supress pixel
							imgnm->item[i] = 0;
						}

					}
				}

				dl_matrix3du_free(imggrad);		// Gradients will not be used any further
				dl_matrix3du_free(imggraddir);

			}





			Serial.printf("[get_edged_shoot] Frame buffer from camera fb size is %u bytes\r\n", fb->len);

			// Sobel operator matrixes
		    int GX[3][3]={{-1, 0, 1},
		                 {-2, 0, 2},
		                 {-1, 0, 1}};
		    int GY[3][3]={{ 1, 2, 1},
		                 { 0, 0, 0},
		                 {-1,-2,-1}};

			dl_matrix3du_t *sobel_matrix_dx = dl_matrix3du_alloc(1, fb->width, fb->height, 1);		// sobel transformation results (dX part)
			if (!sobel_matrix_dx) {
			  Serial.println("[get_edged_shoot] dl_matrix3du_alloc failed");
			}
			else{
			  // do Sobel transform
			  for (uint16_t y = 0; y < fb->height; y++)
			  for (uint16_t x = 0; x < fb->width; x++) {

				  uint32_t i = (y * fb->width + x); // fb->buf address from coordinates





				  image_matrix->item[i*3+0] = (uint8_t) t;
				  image_matrix->item[i*3+1] = (uint8_t) t;
				  image_matrix->item[i*3+2] = (uint8_t) t;
			  }
			  size_t out_len = image_matrix->w * image_matrix->h * 3;
			  free(jpeg_Out->buf);
			  jpeg_Out->buf = NULL;
			  if(!fmt2jpg(image_matrix->item, out_len, image_matrix->w, image_matrix->h, PIXFORMAT_RGB888, 80, &jpeg_Out->buf, &jpeg_Out->buf_len)){
				  Serial.println("[HDR_2_jpeg] JPEG compression failed");
			  }
			  else{
				  Serial.printf("[HDR_2_jpeg] JPEG compression done. Jpeg size is %i bytes\r\n", jpeg_Out->buf_len);
			  }
			}
			dl_matrix3du_free(image_matrix);
			image_matrix = NULL;



	  }










	  //Get required number of frames from camera and integrate desired region to HDR grayscale 16 bit image
	  if(sum_frames(&frame_buf, true, &area_frame, V[V_number_of_sum_frames])){
		  return;
	  }
	  Serial.printf("HDR image: Max bright pixel =%d, Min bright pixel=%d\r\n", frame_buf.max, frame_buf.min);

	  frame window_out;
	  window_out.X1 = 0;
	  window_out.Y1 = 0;
	  window_out.X2 = frame_buf.width;
	  window_out.Y2 = frame_buf.height;

	  Bitmap->Clear();

	  if(ESP_OK != Bitmap->HDR2Bitmap(&frame_buf, &window_out)){
		  Serial.println(PSTR("[takeNewPhoto] HDR2Bitmap failed"));
		  return;
	  }
	  if(ESP_OK != Bitmap->Bitmap2JPEG(&jpeg_output01, 70)){
		  Serial.println(PSTR("[takeNewPhoto] 1 Bitmap2JPEG failed"));
		  return;
	  }

	  refreshCalcs = true;
	}


	if (refreshCalcs) { // recalculate stored image
	  uint16_t offset=0;
	  frame window_out;

	  refreshCalcs = false;

	  //найти средний уровень пикселей окна табло
	  pixel_level = find_middle_level_image(&frame_buf, true);

	  //поиск положения окна цифр (top and bottom edges coordinates) - при найденом уровне по оси y
	  offset = V[V_level_find_digital_Y]*(frame_buf.max-frame_buf.min)/100;
	  if(find_digits_y(&frame_buf, pixel_level + offset, true, &read_window)){ //уровень повысим на 15 единиц, чтобы убрать засветку
		  return;
	  }
	  //поиск положения окна цифр (left and right edges) - between found up and bottom edges
	  if(find_digits_x(&frame_buf, pixel_level + offset, true, &read_window)){
		  return;
	  }
	  Serial.printf("Detected read_window (%d;%d)-(%d;%d)\r\n", read_window.X1, read_window.Y1,read_window.X2, read_window.Y2);


	  //поиск максимума - предположительно середины цифр
	  // find maximum brightness of summary columns values
	  if(find_max_digital_X(&frame_buf, pixel_level, V[V_level_find_digital_X], true, &read_window)){ //уровень повысим на 7 единиц, чтобы убрать засветку
		  return;
	  }

				// Draw detected coordinates output
				window_out.X1 = 0;
				window_out.Y1 = frame_buf.height;
				window_out.X2 = frame_buf.width;
				window_out.Y2 = frame_buf.height * 2;

				if(ESP_OK != Bitmap->HDR2Bitmap(&frame_buf, &window_out)){
				  Serial.println(PSTR("[refreshCalcs] HDR2Bitmap failed"));
				  return;
				}

				// draw rectangle box
				int x = (int)(read_window.X1)						;
				int y = (int)(read_window.Y1)						;
				int w = (int)(read_window.X2 - read_window.X1 + 1)	;
				int h = (int)(read_window.Y2 - read_window.Y1 + 1)	;

				y += window_out.Y1;

				fb_gfx_drawFastHLine(Bitmap->thisPtr, 0, y, 	frame_buf.width, 	FACE_COLOR_GREEN);	//draw read_window borders
				fb_gfx_drawFastHLine(Bitmap->thisPtr, 0, y+h-1, frame_buf.width, 	FACE_COLOR_GREEN);
				fb_gfx_drawFastVLine(Bitmap->thisPtr, x, 	 frame_buf.height, frame_buf.height,	FACE_COLOR_GREEN);
				fb_gfx_drawFastVLine(Bitmap->thisPtr, x+w-1, frame_buf.height, frame_buf.height, 	FACE_COLOR_GREEN);

				h += 4;
				y -= 2;

				for (uint8_t dig = 0; dig < number_letter; dig++) {

					x = max_letter_x[dig];

					fb_gfx_drawFastVLine(Bitmap->thisPtr, x, y, h, 	FACE_COLOR_RED);									// Character center
					fb_gfx_drawFastVLine(Bitmap->thisPtr, x + (Hemming[dig].x_width >> 1), y, h, 	FACE_COLOR_YELLOW);	// char borders
					fb_gfx_drawFastVLine(Bitmap->thisPtr, x - (Hemming[dig].x_width >> 1), y, h, 	FACE_COLOR_YELLOW);

				}


				if(ESP_OK != Bitmap->Bitmap2JPEG(&jpeg_output01, 70)){
					Serial.println(PSTR("[refreshCalcs] 2 Bitmap2JPEG failed"));
					return;
				}

	  //найти средний уровень для каждой цифры
	  find_middle_britnes_digital(&frame_buf, true, &read_window);

	  //отображение на дисплеи

	  window_out.Y1 = frame_buf.height * 2;
	  window_out.Y2 = frame_buf.height * 3;
	  dispalay_ttf_B_W(&frame_buf, V[V_level_convert_to_32], Bitmap->thisPtr, &window_out);//&jpeg_display01); //повысим на 5-20 единиц, чтобы убрать засветку



				if(ESP_OK != Bitmap->Bitmap2JPEG(&jpeg_output01, 70)){
					Serial.println(PSTR("[refreshCalcs] 3 Bitmap2JPEG failed"));
					return;
				}



	  //преобразование в 32 битное числа
	  convert_to_32(&frame_buf, pixel_level, V[V_level_convert_to_32], false, &read_window); //уровень повысим на 20 единиц, чтобы убрать засветку

	  //сравнить с эталоном - рассчет расстояния Хемминга
	  uint8_t count = 0;
	  for (uint8_t dig = 0; dig < number_letter; dig++) { //проврека по всем цифрам шкалы
		  result[count][dig] = image_recognition(dig, V[V_show_digital], &read_window);
		  frequency[result[count][dig]][dig]++; //посчет числа совпадения цифра определенной цифры
	  }

	  for (uint8_t offset_y_test = 0; offset_y_test < min_max_offset_y_test-100; offset_y_test++) { //попробовать смещение по оси Y

		offset_y_current = V[V_offset_y_test] + (offset_y_test - 1); //к уже определенному ранее значению смещения попробовать новое смещение +/-1 или 0

		for (uint8_t dig = 0; dig < number_letter; dig++) { //обнулить массив для поиска частоты повторения цифр
		  for (uint8_t i = 0; i < number_of_samples; i++) { //перебор по всем значения образцов
			frequency[i][dig] = 0;
		  }
		}

		for (uint8_t count = 0; count < average_count; count++) { //повторим результат и найдем опознаные числа

			if (V[V_SH_M3] == 1) print_m3(); //вывести накопленные даные на экран монитора

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

		  Serial.printf("Summary Hemming destination=%d at offset=%d result offset=%.0f\n\n", Sum_min_Hemming[offset_y_test], (offset_y_test - 1), V[V_offset_y_test]);
		}

        //store_check_limits(V[V_level_find_digital_Y], 0, 100, "/V_level_find_digital_Y.txt");
        //store_check_limits(V[V_level_find_digital_X], 0, 150, "/V_level_find_digital_X.txt");
        //store_check_limits(V[V_level_convert_to_32],  0, 150, "/V_level_convert_to_32.txt");

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
		  Serial.printf("Malfunction, summary Hemming destination=%.0f errors total=%.0f\n", V[V_Sum_min_Hemming], V[V_Sum_min_Hemming_error]);
		}
		else {
		  V[V_offset_y_test] +=  (Sum_min_offset_y_test - 1); //запомним лучшее значение
		}

		Serial.printf("Result summary Hemming destination=%d at offset=%d result offset=%.0f errors: %.0f\n\n", Sum_min, Sum_min_offset_y_test - 1, V[V_offset_y_test], V[V_Sum_min_Hemming_error]);
	  }

	}

	if (WiFi.status() != WL_CONNECTED) {
		WiFi_Lost++;
		Serial.printf("No wifi connection %d\n", WiFi_Lost);
	}
	else WiFi_Lost = 0;
	if (WiFi_Lost == 6) WiFi_Connect(); //если нет связи около 4 минут пересоединиться

	delay(1000);

}
//---------------------------------------------------- loop



// From https://github.com/Bodmer/JPEGDecoder/blob/master/examples/Other%20libraries/NodeMCU_Jpeg_ESP/JPEG_functions.ino
// Return the minimum of two values a and b
#define minimum(a,b)     (((a) < (b)) ? (a) : (b))

//====================================================================================
//   Print information decoded from the Jpeg image
//====================================================================================
void jpegInfo() {

  Serial.println("===============");
  Serial.println("JPEG image info");
  Serial.println("===============");
  Serial.print  ("Width      :"); Serial.println(JpegDec.width);
  Serial.print  ("Height     :"); Serial.println(JpegDec.height);
  Serial.print  ("Components :"); Serial.println(JpegDec.comps);
  Serial.print  ("MCU / row  :"); Serial.println(JpegDec.MCUSPerRow);
  Serial.print  ("MCU / col  :"); Serial.println(JpegDec.MCUSPerCol);
  Serial.print  ("Scan type  :"); Serial.println(JpegDec.scanType);
  Serial.print  ("MCU width  :"); Serial.println(JpegDec.MCUWidth);
  Serial.print  ("MCU height :"); Serial.println(JpegDec.MCUHeight);
  Serial.println("===============");
  Serial.println("");
}

//====================================================================================
//   Decode and render the Jpeg image onto the TFT screen
//====================================================================================
void jpegRender(HDR *DestBuff, frame *area_frame) {

  // Only upper left corner used from area_frame - size of area already defined by DestBuff size

  // retrieve infomration about the image
  uint16_t  *pImg;
  uint16_t mcu_w = JpegDec.MCUWidth;
  uint16_t mcu_h = JpegDec.MCUHeight;
  uint32_t max_x = JpegDec.width;
  uint32_t max_y = JpegDec.height;

  // Jpeg images are draw as a set of image block (tiles) called Minimum Coding Units (MCUs)
  // Typically these MCUs are 16x16 pixel blocks
  // Determine the width and height of the right and bottom edge image blocks
  uint32_t min_w = minimum(mcu_w, max_x % mcu_w);
  uint32_t min_h = minimum(mcu_h, max_y % mcu_h);

  // save the current image block size
  uint32_t win_w = mcu_w;
  uint32_t win_h = mcu_h;

  // record the current time so we can measure how long it takes to draw an image
  uint32_t drawTime = millis();

  DestBuff->min = 0xFFFF; // reset min before new frame capture

  // read each MCU block until there are no more
  //while( JpegDec.readSwappedBytes()){ // Swap byte order so the SPI buffer can be used
  while ( JpegDec.read()) { // Normal byte order read

    // save a pointer to the image block
    pImg = JpegDec.pImage;

    // calculate where the image block should be drawn on the screen
    int mcu_x = JpegDec.MCUx * mcu_w;
    int mcu_y = JpegDec.MCUy * mcu_h;

    // check if the image block size needs to be changed for the right and bottom edges
    if (mcu_x + mcu_w <= max_x) win_w = mcu_w;
    else win_w = min_w;
    if (mcu_y + mcu_h <= max_y) win_h = mcu_h;
    else win_h = min_h;

    // check if current MCU are even partly in area_frame bounds. MCU's pixels out of area_frame will not be proceed
    bool inFrame=true;

    inFrame &= ( mcu_x + win_w) 	>= area_frame->X1;						// MCU Right bound
    inFrame &= ( mcu_y + win_h) 	>= area_frame->Y1;						// MCU Bottom bound
    inFrame &= mcu_x				<= area_frame->X1 + DestBuff->width;	// MCU Left bound check
    inFrame &= mcu_y				<= area_frame->Y1 + DestBuff->height;	// MCU Top bound

    // draw image MCU block only if it will fit on the screen
    if (inFrame){

    	//Serial.printf("[inFrame] MCU coodinates are %i:%i/r/n", mcu_x, mcu_y);


		for (uint16_t y = mcu_y; y < mcu_y + win_h; y++)
		for (uint16_t x = mcu_x; x < mcu_x + win_w; x++) {
			// it is X and Y coordinates of JPEG image

			inFrame=true;
			inFrame &= x >= area_frame->X1;						// DestBuff Right bound
			inFrame &= y >= area_frame->Y1;						// DestBuff Bottom bound
			inFrame &= x <  area_frame->X1 + DestBuff->width;	// DestBuff Left bound check
			inFrame &= y <  area_frame->Y1 + DestBuff->height;	// DestBuff Top bound
			// draw only DestBuff area pixels
			if (inFrame){

			  uint32_t i = ((y-mcu_y) * mcu_w + (x-mcu_x)); 							// pImg adress from coordinates
			  uint32_t j = ((y-area_frame->Y1) * DestBuff->width + (x-area_frame->X1)); // DestBuff adress from coordinates

			  //берем 16 битный RGB565 буфер и преобразуем в 16 битный GRAYSCALE
			  //https://stackoverflow.com/questions/58449462/rgb565-to-grayscale
			  int16_t pixel = pImg[i];
			  int16_t red   = ((pixel & 0xF800)>>11);
			  int16_t green = ((pixel & 0x07E0)>>5);
			  int16_t blue  = ( pixel & 0x001F);
			  int16_t grayscale = (0.2126 * red) + (0.7152 * green / 2.0) + (0.0722 * blue);

			  DestBuff->buf[j] += grayscale; // accumulate in HDR

			  DestBuff->max = DestBuff->max > DestBuff->buf[j] ? DestBuff->max : DestBuff->buf[j];
			  DestBuff->min = DestBuff->min < DestBuff->buf[j] ? DestBuff->min : DestBuff->buf[j];

			}
		}

    }
    else if ( ( mcu_y ) > (area_frame->Y1 + DestBuff->height)) JpegDec.abort();

  }
  Serial.printf("[jpegRender] Max bright pixel =%d, Min bright pixel=%d\r\n", DestBuff->max, DestBuff->min);

  // calculate how long it took to draw the image
  drawTime = millis() - drawTime; // Calculate the time it took

  // print the results to the serial port
  Serial.print  ("[jpegRender] Total render time was    : "); Serial.print(drawTime); Serial.println(" ms");

}

