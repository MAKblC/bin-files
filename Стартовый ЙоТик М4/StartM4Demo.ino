/*
**********************************************************************
  тестовый код для  проверки всех устройств "Стартовый" ЙоТик М4
  смотреть в локальной сети по адресу 192.168.4.4
  Ядро esp32 v2.0.1 GyverPortal v3.5.3
  Created by Dmitry Baev
  //********************************************************************
*/

#include <GyverPortal.h>
GyverPortal ui;
// Задаем статический IP-адрес:
IPAddress local_IP(192, 168, 4, 4);
// Задаем IP-адрес сетевого шлюза:
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

#include <EEPROM.h>
#define EEPROM_SIZE 100

#include <Wire.h>
bool TCA9548A;

#include <Adafruit_MCP9808.h>
#include "TLC59108.h"
#include <PCA9634.h>

#include <BH1750.h>    // добавляем библиотеку датчика освещенности // adding Light intensity sensor library
BH1750 LightSensor_1;  // BH1750

#include <Adafruit_Sensor.h>  // добавляем библиотеку датчика температуры, влажности и давления // adding Temp Hum Bar sensor library
#include <Adafruit_BME280.h>  // BME280
Adafruit_BME280 bme280;       //

#define HW_RESET_PIN 0  // Только програмнный сброс
#define I2C_ADDR TLC59108::I2C_ADDR::BASE
//TLC59108 leds(I2C_ADDR + 7);  // Без перемычек добавляется 3 бита адреса
TLC59108 leds(I2C_ADDR + 0);  // когда стоят 3 перемычки

PCA9634 testModule(0x1C);

int mcp3221 = 1;

#include "MCP3221.h"
const byte DEV_ADDR_4E = 0x4E;  // (или 0x4D, 0x49) - настройки датчика температуры и влажности почвы
const byte DEV_ADDR_4D = 0x4D;
const byte DEV_ADDR_49 = 0x49;
MCP3221 mcp3221_4E(DEV_ADDR_4E);
MCP3221 mcp3221_4D(DEV_ADDR_4D);
MCP3221 mcp3221_49(DEV_ADDR_49);
int a = 2312;
int b = 1165;
Adafruit_MCP9808 tempsensor = Adafruit_MCP9808();

#include "PCA9536.h"  // Выходы реле
PCA9536 pca9536;

//Переменные для GP
bool pumpStat = 0;
int temp = 0;
int hum = 0;
int light = 0;
int humA = 0;
int tempA = 0;
int press = 0;
GPcolor colorState;

char* AP_SSID;
int number = 0;

#define I2C_HUB_ADDR 0x70  // настройки I2C для платы MGB-I2C63EN
#define EN_MASK 0x08
#define DEF_CHANNEL 0x00
#define MAX_CHANNEL 0x08

// I2C порт 0x07 - выводы gp16 (SDA), gp17 (SCL)
// I2C порт 0x06 - выводы gp4 (SDA), gp13 (SCL)
// I2C порт 0x05 - выводы gp14 (SDA), gp15 (SCL)
// I2C порт 0x04 - выводы gp5 (SDA), gp23 (SCL)
// I2C порт 0x03 - выводы gp19 (SDA), gp18 (SCL)
//////////////////////////////////////////НАСТРОЙКИ/CONFIGURATION/////////////////////////////////////////////////////////////////

void build() {
  GP.BUILD_BEGIN(900);  // ширина колонки

  GP.THEME(GP_DARK);
  //GP.THEME(GP_LIGHT);

  GP.UPDATE("lightGH,tGH,humGH,taGH,humaGH,pres");

  GP_MAKE_BLOCK_TAB(
    "Умная теплица",
    GP_MAKE_BOX(GP_CENTER,
                GP_MAKE_BLOCK_THIN_TAB(
                  "Насос",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL("Выкл"); GP.SWITCH("pump", pumpStat); GP.LABEL("Вкл");););

    );

    GP_MAKE_BOX(GP_CENTER,
                GP_MAKE_BLOCK_THIN_TAB(
                  "Освещение",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL("Выбрать цвет"); GP.COLOR("colorGH", colorState);););

                GP_MAKE_BLOCK_THIN_TAB(
                  "Освещенность",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL_BLOCK("0", "lightGH"); GP.LABEL("lx");););

    );

    GP_MAKE_BOX(GP_CENTER,
                GP_MAKE_BLOCK_THIN_TAB(
                  "Температура почвы",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL_BLOCK("0", "tGH"); GP.LABEL("°C");););
                GP_MAKE_BLOCK_THIN_TAB(
                  "Влажность почвы",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL_BLOCK("0", "humGH"); GP.LABEL("%");););

    );

    GP_MAKE_BOX(GP_CENTER,
                GP_MAKE_BLOCK_THIN_TAB(
                  "Температура воздуха",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL_BLOCK("0", "taGH"); GP.LABEL("°C");););
                GP_MAKE_BLOCK_THIN_TAB(
                  "Влажность воздуха",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL_BLOCK("0", "humaGH"); GP.LABEL("%");););
                GP_MAKE_BLOCK_THIN_TAB(
                  "Давление",
                  GP_MAKE_BOX(GP_CENTER, GP.LABEL_BLOCK("0", "pres"); GP.LABEL("hPa");););

    );

  );

  GP.BUILD_END();
}
void action() {
  if (ui.click()) {
    if (ui.clickBool("pump", pumpStat)) {
      setBusChannel(0x03);
      pumpStat == true ? pca9536.setState(IO1, IO_LOW) : pca9536.setState(IO1, IO_HIGH);
    }

    if (ui.clickColor("colorGH", colorState)) {
      setBusChannel(0x06);

      leds.setBrightness(3, colorState.r);
      leds.setBrightness(2, colorState.g);
      leds.setBrightness(5, colorState.b);

      for (int channel = 0; channel < testModule.channelCount(); channel++) {
        testModule.setLedDriverMode(channel, PCA9634_LEDPWM);  // установка режима ШИМ (0-255)
      }

      testModule.write1(3, colorState.r);
      testModule.write1(2, colorState.g);
      testModule.write1(5, colorState.b);
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
    if (ui.update("humaGH")) {
      ui.answer(tempA);
    }
    if (ui.update("taGH")) {
      ui.answer(humA);
    }
    if (ui.update("pres")) {
      ui.answer(press);
    }
  }
}

void checkMCP() {
  setBusChannel(0x04);

  mcp3221_4E.setAlpha(DEFAULT_ALPHA);
  mcp3221_4E.setNumSamples(DEFAULT_NUM_SAMPLES);
  mcp3221_4E.setSmoothing(ROLLING_AVG);
  delay(100);
  hum = mcp3221_4E.getData();
  hum = map(hum, a, b, 0, 100);
  Serial.print(hum);
  Serial.println(" 4E");
  if (hum != 202 && hum != -202) {
    mcp3221 = 1;
    return;
  }

  mcp3221_4D.setAlpha(DEFAULT_ALPHA);
  mcp3221_4D.setNumSamples(DEFAULT_NUM_SAMPLES);
  mcp3221_4D.setSmoothing(ROLLING_AVG);
  hum = mcp3221_4D.getData();
  hum = map(hum, a, b, 0, 100);
  Serial.print(hum);
  Serial.println(" 4D");
  if (hum != 202 && hum != -202) {
    mcp3221 = 2;
    return;
  }

  mcp3221_49.setAlpha(DEFAULT_ALPHA);
  mcp3221_49.setNumSamples(DEFAULT_NUM_SAMPLES);
  mcp3221_49.setSmoothing(ROLLING_AVG);
  hum = mcp3221_49.getData();
  hum = map(hum, a, b, 0, 100);
  Serial.print(hum);
  Serial.println(" 49");
  if (hum != 202 && hum != -202) {
    mcp3221 = 3;
    return;
  }
}

void setup() {
  Serial.begin(115200);
  delay(512);
  Wire.begin();
  setI2Cmodule();

  checkMCP();

  if (!tempsensor.begin(0x18)) {
    Serial.println("Couldn't find MCP9808!");
  }
  tempsensor.setResolution(3);

  setBusChannel(0x03);
  pca9536.reset();
  pca9536.setMode(IO_OUTPUT);
  pca9536.setState(IO1, IO_HIGH);
  pca9536.setState(IO0, IO_HIGH);

  setBusChannel(0x06);
  //Wire.setClock(10000L);

  leds.init(HW_RESET_PIN);
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  byte pwm = 0;
  leds.setAllBrightness(pwm);

  testModule.begin();
  for (int channel = 0; channel < testModule.channelCount(); channel++) {
    testModule.setLedDriverMode(channel, PCA9634_LEDOFF);  // выключить все светодиоды в режиме 0/1
  }
  for (int channel = 0; channel < testModule.channelCount(); channel++) {
    testModule.setLedDriverMode(channel, PCA9634_LEDPWM);  // установка режима ШИМ (0-255)
  }

  LightSensor_1.begin();  // запуск датчика освещенности // turn the light intensity sensor on

  setBusChannel(0x07);
  bool bme_status = bme280.begin();
  if (!bme_status) {
    Serial.println("Не найден по адресу 0х77, пробую другой...");
    bme_status = bme280.begin(0x76);
    if (!bme_status)
      Serial.println("Датчик не найден, проверьте соединение");
  }
  EEPROM.begin(EEPROM_SIZE);
  checkNamber();

  // Инициируем WiFi
  WiFi.mode(WIFI_AP);
  Serial.print("Настоить статический IP: ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Удалось" : "Не удалось!");
  WiFi.softAP(AP_SSID);
  Serial.print("IP для подключения: ");
  Serial.println(WiFi.softAPIP());

  // подключаем конструктор и запускаем
  ui.attachBuild(build);
  ui.attach(action);
  ui.start();
  Serial.println("Веб страница запущена");
}

void checkNamber() {
  EEPROM.get(95, number);
  Serial.println(number);
  if (number == -1) {
    number = random(10000);
    EEPROM.put(95, number);
    EEPROM.commit();
  }

  String n = "StartoviyM4_№-";
  n = n + String(number);

  AP_SSID = strcpy(new char[n.length() + 1], n.c_str());
}

static uint32_t tmr;
void loop() {
  ui.tick();

  if (millis() - tmr >= 500) {
    Serial.println("Wifi: " + String(AP_SSID) + " / Адрес: " + WiFi.softAPIP().toString());
    tmr = millis();
    light = LightSensor_1.readLightLevel();
    Serial.println(String(light) + " lx");
    setBusChannel(0x04);
    tempsensor.wake();
    temp = tempsensor.readTempC();
    Serial.println("Soil temperature = " + String(temp) + " *C");
    if (mcp3221 == 1) {
      hum = mcp3221_4E.getData();
    }else{
      if(mcp3221 == 2){
        hum = mcp3221_4D.getData();
      }
      else{
        hum = mcp3221_49.getData();
      }
    }
    hum = map(hum, a, b, 0, 100);
    Serial.println("Soil humidity = " + String(hum) + " %");
    setBusChannel(0x07);
    tempA = bme280.readTemperature();
    humA = bme280.readHumidity();
    press = bme280.readPressure() / 100.0F;
    Serial.print("Air temperature = ");
    Serial.println(String(tempA) + " C");
    Serial.print("Air humidity = ");
    Serial.println(String(humA) + " %");
    Serial.print("Air pressure = ");
    Serial.println(String(press) + " hPa");
  }
}

//функция смены I2C-порта
int setI2Cmodule(void) {
  int error;
  /*Контроллер подключается к адресу платы расширения*/
  Wire.beginTransmission(I2C_HUB_ADDR);
  delay(100);
  error = Wire.endTransmission();  //если плата расширения не найдена или не определена - контроллер выходит из функции определения с выведением ошибки.
  if (error != 0) {
    Serial.print("\nNo I2C Module / Broken Module\n");
    Serial.print("Error #");
    Serial.print(error);
    Serial.print("\n");
    return error;
  }
  /*Контроллер переключает вывод платы расширения на GP17/GP16 по старому образцу*/
  Wire.beginTransmission(I2C_HUB_ADDR);
  delay(100);
  Wire.write(0x07 | EN_MASK);
  Wire.endTransmission();
  Wire.beginTransmission(0x76);  //опрос подключенного к выводу датчика MGS-THP80
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.print("\nOld Type Module");
    TCA9548A = false;
    return error;
  }
  Wire.beginTransmission(0x77);  //опрос подключенного к выводу датчика MGS-THP80
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.print("\nOld Type Module");
    TCA9548A = false;
    return error;
  }
  /*Контроллер переключает вывод платы расширения на GP17/GP16 по новому образцу*/
  Wire.beginTransmission(I2C_HUB_ADDR);
  delay(100);
  Wire.write(0x01 << 0x07);
  Wire.endTransmission();
  Wire.beginTransmission(0x76);  //опрос подключенного к выводу датчика MGS-THP80
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.print("\nNew Type Module");
    TCA9548A = true;
    return error;
  }
  Wire.beginTransmission(0x77);  //опрос подключенного к выводу датчика MGS-THP80
  error = Wire.endTransmission();
  if (error == 0) {
    Serial.print("\nNew Type Module");
    TCA9548A = true;
    return error;
  }
  /*если плата расширения обнаружена, но не определился образец модели, выводится ошибка*/
  Serial.print("\nError #");
  Serial.print(error);
  return error;

  /*ERRORS:
    0: success.

    1: data too long to fit in transmit buffer.
    2: received NACK on transmit of address.
    3: received NACK on transmit of data.
    4: other error.
    5: timeout
  */
}

/*переключение выводов платы расширения*/
bool setBusChannel(uint8_t i2c_channel) {
  if (i2c_channel >= MAX_CHANNEL) {
    return false;
  } else {
    Wire.beginTransmission(I2C_HUB_ADDR);
    /*определена плата расширения модели TI TCA9548A (нового образца)*/
    if (TCA9548A == true) {
      Serial.println("New TI TCA9548A module");
      Wire.write(0x01 << i2c_channel);
    }
    /*в противном случае считается, что модель платы расширения - NXP PSA9547PW (старого образца)*/
    else {
      Serial.println("Old NXP PCA9547PW module");
      Wire.write(i2c_channel | EN_MASK);
    }

    Wire.endTransmission();
    return true;
  }
}