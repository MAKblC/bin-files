#define AP_SSID "IoT32_G_H"
#define AP_PASS ""

#include <GyverPortal.h>
GyverPortal ui;
// Задаем статический IP-адрес:
IPAddress local_IP(192, 168, 4, 4);
// Задаем IP-адрес сетевого шлюза:
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

#include <Adafruit_ADS1015.h>
#include <Wire.h>
#include <ESP32_Servo.h>  // конфигурация сервомотора // servo configuration

#include <FastLED.h>  // конфигурация матрицы // LED matrix configuration
#include <FastLED_GFX.h>
#include <FastLEDMatrix.h>
#define NUM_LEDS 64      // количество светодиодов в матрице // number of LEDs
CRGB leds[NUM_LEDS];     // определяем матрицу (FastLED библиотека) // defining the matrix (fastLED library)
#define LED_PIN 18       // пин к которому подключена матрица // matrix pin
#define COLOR_ORDER GRB  // порядок цветов матрицы // color order
#define CHIPSET WS2812   // тип светодиодов // LED type

#define pump 17  // пин насоса // pump pin
#define wind 16  // пин вентилятора // cooler pin

Servo myservo;
int pos = 1;        // начальная позиция сервомотора // servo start position
int prevangle = 1;  // предыдущий угол сервомотора // previous angle of servo

#include <BH1750.h>    // добавляем библиотеку датчика освещенности // adding Light intensity sensor library
BH1750 LightSensor_1;  // BH1750

#include <Adafruit_Sensor.h>  // добавляем библиотеку датчика температуры, влажности и давления // adding Temp Hum Bar sensor library
#include <Adafruit_BME280.h>  // BME280
Adafruit_BME280 bme280;       //

//Переменные для GP
int widowStat = 0;
bool windStat = 0;
bool pumpStat = 0;
int temp = 0;
int hum = 0;
int light = 0;
GPcolor colorState;
//////////////////////////////////////////НАСТРОЙКИ/CONFIGURATION/////////////////////////////////////////////////////////////////

void build() {
  GP.BUILD_BEGIN(900);  // ширина колонки

  GP.THEME(GP_DARK);

  GP.UPDATE("co2GH,lightGH,tGH,humGH");
  GP_MAKE_BLOCK_TAB(
    "Умная теплица",
    GP_MAKE_BOX(GP_CENTER,
                GP_MAKE_BLOCK_THIN_TAB(
                  "Вентилятор",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL("Выкл"); GP.SWITCH("wind", windStat); GP.LABEL("Вкл");););

                GP_MAKE_BLOCK_THIN_TAB(
                  "Насос",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL("Выкл"); GP.SWITCH("pump", pumpStat); GP.LABEL("Вкл"););););
    GP_MAKE_BOX(GP_CENTER, GP.LED_GREEN("led1", false); GP.LED_GREEN("led2", false); GP.LED_GREEN("led3", false); GP.LED_GREEN("led4", false););
    GP_MAKE_BOX(GP_CENTER,

                GP_MAKE_BLOCK_THIN_TAB(
                  "Форточка",
                  GP_MAKE_BOX(GP_CENTER, GP.SLIDER("window", widowStat, 0, 100);););

                GP_MAKE_BLOCK_THIN_TAB(
                  "Освещение",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL("Выбрать цвет"); GP.COLOR("colorGH", colorState););););

    GP_MAKE_BOX(GP_CENTER, GP_MAKE_BLOCK_THIN_TAB(
                  "Температура",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL_BLOCK("0", "tGH"); GP.LABEL("°C");););
                GP_MAKE_BLOCK_THIN_TAB(
                  "Влажность",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL_BLOCK("0", "humGH"); GP.LABEL("%");););
                GP_MAKE_BLOCK_THIN_TAB(
                  "Освещенность",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL_BLOCK("0", "lightGH"); GP.LABEL("lx");););););

  GP.BUILD_END();
}
void action() {
  if (ui.click()) {
    if (ui.clickInt("window", widowStat)) {
      myservo.write(widowStat);
      Serial.println("servo position: ");
      Serial.println(widowStat);
      Serial.println("°");
    }

    if (ui.clickBool("pump", pumpStat)) {
      Serial.println(pumpStat);
      digitalWrite(pump, pumpStat);
    }

    if (ui.clickBool("wind", windStat)) {
      Serial.println(windStat);
      digitalWrite(wind, windStat);
    }

    if (ui.clickColor("colorGH", colorState)) {
      fill_solid(leds, NUM_LEDS, CRGB(colorState.r, colorState.g, colorState.b));  // заполнить всю матрицу выбранным цветом
      FastLED.show();
    }
  }

  if (ui.update()) {
    if (ui.update("lightGH")) {
      ui.answer(light);
    }
    if (ui.update("tGH")) {
      ui.answer(temp);
    }
    if (ui.update("humGH")) {
      ui.answer(hum);
    }
  }
}
void setup() {
  myservo.attach(19);  // пин сервомотора // servo pin
  Wire.begin();
  pinMode(pump, OUTPUT);
  pinMode(wind, OUTPUT);    // настройка пинов насоса и вентилятора на выход // pump and cooler pins configured on output mode
  digitalWrite(pump, LOW);  // устанавливаем насос и вентилятор изначально выключенными // turn cooler and pump off
  digitalWrite(wind, LOW);

  Serial.begin(115200);
  delay(512);

  // Инициируем WiFi
  WiFi.mode(WIFI_AP);
  Serial.print("Настоить статический IP: ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Удалось" : "Не удалось!");
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("IP для подключения: ");
  Serial.println(WiFi.softAPIP());

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);  // конфигурация матрицы // matrix configuration

  LightSensor_1.begin();  // запуск датчика освещенности // turn the light intensity sensor on

  bool bme_status = bme280.begin();
  if (!bme_status)
    Serial.println("Could not find a valid BME280 sensor, check wiring!");  // проверка  датчика температуры, влажности и давления // checking the temp hum bar sensor

  // подключаем конструктор и запускаем
  ui.attachBuild(build);
  ui.attach(action);
  ui.start();
  Serial.println("Веб страница запущена");
}

static uint32_t tmr;
void loop() {
  ui.tick();

  if (millis() - tmr >= 500) {
    tmr = millis();
    light = LightSensor_1.readLightLevel();
    // Вывод измеренных значений в терминал
    Serial.println(String(light, 1) + " lx");
    temp = bme280.readTemperature();
    Serial.println("Air temperature = " + String(temp) + " *C");
    hum = bme280.readHumidity();
    Serial.println("Air humidity = " + String(hum) + " %");
  }
}
