#include <Wire.h> //библиотека для I2C интерфейса
bool TCA9548A;

#include <Adafruit_PWMServoDriver.h> // библиотека для моторной платы
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x70); // адрес платы

#include <VL53L0X.h> // библиотека для датчика MGS-D20
VL53L0X lox1;

// Выберите гироскоп или датчик цвета в вашей сборке (ненужные занесите в комментарии)
#define MGS_A9 1
//#define MGS_CLM60 1
//#define MGS_A6 1
/*
  /////////////////// гироскоп и датчик цвета ///////////////////
  #include <Adafruit_LSM9DS1.h>
  #include <MPU6050.h>
  #include "Adafruit_APDS9960.h"

  #ifdef MGS_A9
  Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();
  #endif
  #ifdef MGS_A6
  MPU6050 mpu;
  #endif
  #ifdef MGS_CLM60
  Adafruit_APDS9960 apds9960;
  #endif
*/
// Выберите модуль светодиодов в вашей сборке (ненужные занесите в комментарии)
//#define MGL_RGB1EN 1
//#define MGL_RGB2 1
//#define MGL_RGB3 1

/////////////////// модуль светодиодов ///////////////////

#include "TLC59108.h" // библиотека для модуля MGL-RGB1
#define HW_RESET_PIN 0 // Только програмнный сброс
#define I2C_ADDR TLC59108::I2C_ADDR::BASE
TLC59108 leds(I2C_ADDR + 0); // Без перемычек добавляется 3 бита адреса

#include <PCA9634.h>
PCA9634 testModule(0x1C); // (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)

#define I2C_HUB_ADDR        0x70 // настройки I2C для платы MGB-I2C63EN
#define EN_MASK             0x08
#define DEF_CHANNEL         0x00
#define MAX_CHANNEL         0x08

/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/

#define LONG_RANGE // режим дальности для датчика расстояния

#include <Adafruit_MCP4725.h>    // библиотека для MGB-BUZ1
Adafruit_MCP4725 buzzer;
int ton;
int vol1 = 4095; // Уровень громкости = vol1-vol2
int vol2 = 0;

// пример с удерживанием кнопки
const char* AP_SSID = "Dynamica";
const char* AP_PASS = "";

#include <GyverPortal.h>
GyverPortal portal;
IPAddress local_IP(192, 168, 4, 22);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

bool StLight;   // свет
bool StColor;   // цвет
bool StUV;   // свет
bool holdNoise; // гудок
bool holdFwr;  // вперёд
bool holdBwr;  // назад
bool holdLft;  // влево
bool holdRgh;  // вправо

String dist = "0"; // с датчика расстояния
/*
  //C гироскопа
  String x = "0";
  String y = "0";
  String z = "0";
  String r = "0";
  String g = "0";
  String b = "0";
*/
int power = 0; // скорость
int correction = 0; // корректировка баланса колёс
int brLight = 100;
int brUV = 100;

GPcolor valColor;
GPcolor valColorSens;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  setI2Cmodule();

  // подключаем конструктор и запускаем
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(AP_SSID);
  portal.attachBuild(build);
  portal.attach(action);
  portal.start();

  // Инициализация драйвера
  pwm.begin();
  // Частота (Гц)
  pwm.setPWMFreq(100);
  // Все порты выключены
  pwm.setPWM(8, 0, 4096);
  pwm.setPWM(9, 0, 4096);
  pwm.setPWM(10, 0, 4096);
  pwm.setPWM(11, 0, 4096);
  /*
    setBusChannel(0x06);
    #ifdef MGS_A9
    if (!lsm.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
    }
    Serial.println("Found LSM9DS1 9DOF");
    lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
    lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
    lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
    #endif
    #ifdef MGS_A6
    if (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G, 0x69))
    {
      Serial.println("MGS_A6 Не обнаружен! Проверьте адрес!"); // (также попробуйте просканировать адрес: https://github.com/MAKblC/Codes/tree/master/I2C%20scanner)
      delay(500);
    }
    Serial.println("A6");
    // Calibrate gyroscope. The calibration must be at rest.
    // If you don't want calibrate, comment this line.
    mpu.calibrateGyro();

    // Set threshold sensivty. Default 3.
    // If you don't want use threshold, comment this line or set 0.
    mpu.setThreshold(3);

    // Check settings
    Serial.println();

    Serial.print(" * Sleep Mode:        ");
    Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");

    Serial.print(" * Clock Source:      ");
    switch (mpu.getClockSource())
    {
      case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
      case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
      case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
      case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
      case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
      case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
      case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
    }

    Serial.print(" * Gyroscope:         ");
    switch (mpu.getScale())
    {
      case MPU6050_SCALE_2000DPS:        Serial.println("2000 dps"); break;
      case MPU6050_SCALE_1000DPS:        Serial.println("1000 dps"); break;
      case MPU6050_SCALE_500DPS:         Serial.println("500 dps"); break;
      case MPU6050_SCALE_250DPS:         Serial.println("250 dps"); break;
    }

    Serial.print(" * Gyroscope offsets: ");
    Serial.print(mpu.getGyroOffsetX());
    Serial.print(" / ");
    Serial.print(mpu.getGyroOffsetY());
    Serial.print(" / ");
    Serial.println(mpu.getGyroOffsetZ());

    Serial.println();
    #endif
    #ifdef MGS_CLM60
    if (!apds9960.begin()) {
      Serial.println("Failed to initialize device!");
    }
    Serial.println("CLM60");
    // Инициализация режимов работы датчика
    apds9960.enableColor(true);
    apds9960.enableProximity(true);
    #endif
  */
  setBusChannel(0x07); // 7ой канал
  buzzer.begin(0x60); // С перемычкой адрес будет 0x60
  buzzer.setVoltage(0, false);   // выключение звука

  setBusChannel(0x05); //5ый канал
  lox1.init();
  lox1.setTimeout(500);
#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  lox1.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  lox1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  lox1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  lox1.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  lox1.setMeasurementTimingBudget(200000);
#endif

  setBusChannel(0x01); // первый канал
  leds.init(HW_RESET_PIN);
  leds.setLedOutputMode(TLC59108::LED_MODE::PWM_IND);
  testModule.begin();
  for (int channel = 0; channel < testModule.channelCount(); channel++)
  {
    testModule.setLedDriverMode(channel, PCA9634_LEDOFF); // выключить все светодиоды в режиме 0/1
    testModule.setLedDriverMode(channel, PCA9634_LEDPWM); // установка режима ШИМ
  }
}

void build() {
  GP.BUILD_BEGIN();
  GP.THEME(GP_DARK);

  GP.UPDATE("distance");

  GP_MAKE_BLOCK_THIN_TAB(
    "Джойстик движения",
    GP_MAKE_BOX(
      GP_CENTER,
      GP.BUTTON_MINI("btnFwr", "Вперёд");
    );
    GP_MAKE_BOX(
      GP_CENTER,
      GP.BUTTON_MINI("btnLft", "Влево");
      GP.BUTTON_MINI("btnNoise", "Гудок");
      GP.BUTTON_MINI("btnRgh", "Вправо");
    );
    GP_MAKE_BOX(
      GP_CENTER,
      GP.BUTTON_MINI("btnBwr", "Назад");
    );
  );

  GP_MAKE_SPOILER(
    "Функции света",
    GP_MAKE_BLOCK_THIN_TAB("Свет",
                           GP.LABEL_BLOCK("off");
                           GP.SWITCH("swLight", StLight);
                           GP.LABEL_BLOCK("on"););

    GP_MAKE_BLOCK_THIN_TAB("Цвет",
                           GP.LABEL_BLOCK("off");
                           GP.SWITCH("swColor", StColor);
                           GP.LABEL_BLOCK("on"););
    GP_MAKE_BLOCK_THIN_TAB("Ултра Фиолет",
                           GP.LABEL_BLOCK("off");
                           GP.SWITCH("swUV", StUV);
                           GP.LABEL_BLOCK("on"););
  );

  GP.HR();

  GP_MAKE_SPOILER(
    "Настройки",
    GP_MAKE_BOX( GP.LABEL("Скорость");  GP.SLIDER("pwr", power, 0, 100); );
    GP.HR();
    GP_MAKE_BOX( GP_CENTER, GP.LABEL("Корректировка мощности моторов"); );
    GP_MAKE_BOX( GP.LABEL("←");  GP.SPINNER("corr", correction, -50, 50); GP.LABEL("→"));
    GP.HR();
    GP_MAKE_BOX( GP.LABEL("Яркость белого");  GP.SLIDER("slBrLig", brLight, 0, 255); );
    GP_MAKE_BOX( GP.LABEL("Яркость UV");  GP.SLIDER("slBrUV", brUV, 0, 255); );
    GP_MAKE_BOX( GP.LABEL("Выбор цвета RGB");   GP.COLOR("col", valColor););
  );

  GP.HR();

  GP_MAKE_SPOILER(
    "Датчики",
    GP_MAKE_BOX( GP.LABEL("Расстояние");  GP.LABEL_BLOCK(dist, "distance");  );

    GP.HR();
    /*#ifdef MGS_A9
        GP_MAKE_BOX( GP.LABEL("Акс-метр");
                     GP.LABEL_BLOCK(x, "giroX");
                     GP.LABEL_BLOCK(y, "giroY");
                     GP.LABEL_BLOCK(z, "giroZ");
                   );
      #endif
      #ifdef MGS_A6
        GP_MAKE_BOX( GP.LABEL("Гироскоп");
                     GP.LABEL_BLOCK(x, "giroX");
                     GP.LABEL_BLOCK(y, "giroY");
                     GP.LABEL_BLOCK(z, "giroZ");
                   );
      #endif
      #ifdef MGS_CLM60
        GP_MAKE_BOX( GP.LABEL("Датчик цвета");
                     GP.COLOR("colSens", valColorSens, true);
                   );
        GP_MAKE_BOX(
          GP.LABEL_BLOCK(r, "cR");
          GP.LABEL_BLOCK(g, "cG");
          GP.LABEL_BLOCK(b, "cB");
        );
      #endif*/
  );

  GP.BUILD_END();
}

void action() {
  if (portal.clickDown("btnFwr")) holdFwr = 1;
  if (portal.clickUp("btnFwr")) holdFwr = 0;
  if (portal.clickDown("btnBwr")) holdBwr = 1;
  if (portal.clickUp("btnBwr")) holdBwr = 0;
  if (portal.clickDown("btnLft")) holdLft = 1;
  if (portal.clickUp("btnLft")) holdLft = 0;
  if (portal.clickDown("btnRgh")) holdRgh = 1;
  if (portal.clickUp("btnRgh")) holdRgh = 0;
  if (portal.clickDown("btnNoise")) holdNoise = 1;
  if (portal.clickUp("btnNoise")) holdNoise = 0;

  // было обновление
  if (portal.update()) {
    if (portal.update("distance")) portal.answer(dist);
    /* if (portal.update("giroX")) portal.answer(x);
      if (portal.update("giroY")) portal.answer(y);
      if (portal.update("giroZ")) portal.answer(z);
      if (portal.update("cR")) portal.answer(r);
      if (portal.update("cG")) portal.answer(g);
      if (portal.update("cB")) portal.answer(b);
      if (portal.update("colSens")) portal.answer(valColorSens);*/
  }

  if (portal.click()) {
    // проверяем компоненты и обновляем переменные
    if (portal.clickInt("pwr", power)) {
      Serial.print("Скорость: ");
      Serial.println(power);
    }
    if (portal.clickInt("corr", correction)) {
      Serial.print("Коррекция: ");
      Serial.println(correction);
    }
    if (portal.clickInt("slBrLig", brLight)) {
      Serial.print("Яркость белого: ");
      Serial.println(brLight);
    }
    if (portal.clickInt("slBrUV", brUV)) {
      Serial.print("Яркость УФ: ");
      Serial.println(brUV);
    }
    if (portal.clickBool("swLight", StLight)) {
      Serial.print("Свет: ");
      Serial.println(StLight);
    }
    if (portal.clickBool("swColor", StColor)) {
      Serial.print("Цвет: ");
      Serial.println(StColor);
    }
    if (portal.clickBool("swUV", StUV)) {
      Serial.print("УФ: ");
      Serial.println(StUV);
    }
    if (portal.clickColor("col", valColor)) {
      Serial.print("УФ: ");
      Serial.println(StUV);
    }
  }
}


void loop() {

  portal.tick();

  if (holdFwr == 1) {
    Serial.println(power);
    motorA_setpower(power + correction, true);
    motorB_setpower(power - correction, false);
    delay(200);
  } else if (holdBwr == 1) {
    motorA_setpower(power + correction, false);
    motorB_setpower(power - correction, true);
    delay(200);
  } else  if (holdLft == 1) {
    motorA_setpower(power + correction, true);
    motorB_setpower(power - correction, true);
    delay(200);
  } else  if (holdRgh == 1) {
    motorA_setpower(power + correction, false);
    motorB_setpower(power - correction, false);
    delay(200);
  } else {
    motorA_setpower(0, false);
    motorB_setpower(0, true);
  }

  if (holdNoise == 1) {
    setBusChannel(0x07);
    for (int i = 0; i < 500; i++) {
      buzzer.setVoltage(vol1, false);
      buzzer.setVoltage(vol2, false);
      delayMicroseconds(190);
    }

  } else {
    setBusChannel(0x07);
    buzzer.setVoltage(0, false);
  }
  if (StLight == 1) {
    Serial.println(brLight);
    leds.setBrightness(6, brLight); // белые
    leds.setBrightness(0, brLight);
    testModule.write1(6, brLight);
    testModule.write1(0, brLight);
  } else {
    leds.setBrightness(6, 0x00); // белые
    leds.setBrightness(0, 0x00);
    testModule.write1(6, 0x00);
    testModule.write1(0, 0x00);
  }

  if (StUV == 1) {
    leds.setBrightness(1, brUV); // УФ
    leds.setBrightness(4, brUV);
    testModule.write1(1, brUV);
    testModule.write1(4, brUV);
  } else {
    leds.setBrightness(1, 0); // УФ
    leds.setBrightness(4, 0);
    testModule.write1(1, 0);
    testModule.write1(4, 0);
  }

  if (StColor == 1) {
    leds.setBrightness(3, valColor.r); // красный
    leds.setBrightness(2, valColor.g);// зеленый
    leds.setBrightness(5, valColor.b); // синий
    testModule.write1(3, valColor.r);
    testModule.write1(2, valColor.g);
    testModule.write1(5, valColor.b);
  } else {
    leds.setBrightness(3, 0); // красный
    leds.setBrightness(2, 0);// зеленый
    leds.setBrightness(5, 0); // синий
    testModule.write1(3, 0);
    testModule.write1(2, 0);
    testModule.write1(5, 0);
  }

  setBusChannel(0x05);
  dist = String(lox1.readRangeSingleMillimeters()) + " мм"; // снимаем данные с датчика расстояния
  /*
    setBusChannel(0x06);
    #ifdef MGS_A9
    lsm.read(); // данные гироскопа, акселерометра и магнетометра
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);
    x = "X: " + String(a.acceleration.x, 1);
    y = "Y: " + String(a.acceleration.y, 1);
    z = "Z: " + String(a.acceleration.z, 1);
    #endif
    #ifdef MGS_A6
    Vector rawGyro = mpu.readRawGyro(); // Сырые значения
    Vector normGyro = mpu.readNormalizeGyro(); // Преобразованные значения
    x = "X: " + String(normGyro.XAxis, 1);
    y = "Y: " + String(normGyro.YAxis, 1);
    z = "Z: " + String(normGyro.ZAxis, 1);
    #endif
    #ifdef MGS_CLM60
    uint16_t red_data   = 0;
    uint16_t green_data = 0;
    uint16_t blue_data  = 0;
    uint16_t clear_data = 0;
    uint16_t prox_data  = 0;
    // Определение цвета
    while (!apds9960.colorDataReady()) {
      delay(5);
    }
    apds9960.getColorData(&red_data, &green_data, &blue_data, &clear_data);
    // Определение близости препятствия
    prox_data = apds9960.readProximity();
    r = "Red:" + String(red_data);
    g = "Green:" + String(green_data);
    b = "Blue:" + String(blue_data);
    valColorSens.setRGB(red_data, green_data, blue_data);
    Serial.println("RED   = " + String(red_data));
    Serial.println("GREEN = " + String(green_data));
    Serial.println("BLUE  = " + String(blue_data));
    #endif
  */
}

// Мощность мотора "A" от -100% до +100% (от знака зависит направление вращения)
void motorA_setpower(float pwr, bool invert)
{
  // Проверка, инвертирован ли мотор
  if (invert)
  {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100)
  {
    pwr = -100;
  }
  if (pwr > 100)
  {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0)
  {
    pwm.setPWM(10, 0, 4096);
    pwm.setPWM(11, 0, pwmvalue);
  }
  else
  {
    pwm.setPWM(11, 0, 4096);
    pwm.setPWM(10, 0, pwmvalue);
  }
}

// Мощность мотора "B" от -100% до +100% (от знака зависит направление вращения)
void motorB_setpower(float pwr, bool invert)
{
  // Проверка, инвертирован ли мотор
  if (invert)
  {
    pwr = -pwr;
  }
  // Проверка диапазонов
  if (pwr < -100)
  {
    pwr = -100;
  }
  if (pwr > 100)
  {
    pwr = 100;
  }
  int pwmvalue = fabs(pwr) * 40.95;
  if (pwr < 0)
  {
    pwm.setPWM(8, 0, 4096);
    pwm.setPWM(9, 0, pwmvalue);
  }
  else
  {
    pwm.setPWM(9, 0, 4096);
    pwm.setPWM(8, 0, pwmvalue);
  }
}

/*bool setBusChannel(uint8_t i2c_channel) // смена I2C порта
  {
  if (i2c_channel >= MAX_CHANNEL)
  {
    return false;
  }
  else
  {
    Wire.beginTransmission(I2C_HUB_ADDR);
    // Wire.write(i2c_channel | EN_MASK);
    Wire.write(0x01 << i2c_channel); // Для микросхемы PW548A
    Wire.endTransmission();
    return true;
  }
  }*/
int setI2Cmodule(void) {
  int error;
  /*Контроллер подключается к адресу платы расширения*/
  Wire.beginTransmission(I2C_HUB_ADDR);
  delay(100);
  error = Wire.endTransmission(); //если плата расширения не найдена или не определена - контроллер выходит из функции определения с выведением ошибки.
  if (error != 0)
  {
    Serial.print("\nNo I2C Module / Broken Module\n");
    Serial.print("Error #");
    Serial.print(error);
    Serial.print("\n");
    return error;
  }
  /*Контроллер переключает вывод платы расширения на GP14/GP15 по старому образцу*/
  Wire.beginTransmission(I2C_HUB_ADDR);
  delay(100);
  Wire.write(EN_MASK | 0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x29); //опрос подключенного к выводу датчика MGS-D20
  error = Wire.endTransmission();
  if (error == 0)
  {
    Serial.print("\nOld Type Module");
    TCA9548A = false;
    return error;
  }
  /*Контроллер переключает вывод платы расширения на GP14/GP15 по новому образцу*/
  Wire.beginTransmission(I2C_HUB_ADDR);
  delay(100);
  Wire.write(0x01 << 0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x29); //опрос подключенного к выводу датчика MGS-D20
  error = Wire.endTransmission();
  if (error == 0)
  {
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
bool setBusChannel(uint8_t i2c_channel)
{
  if (i2c_channel >= MAX_CHANNEL)
  {
    return false;
  }
  else
  {
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
