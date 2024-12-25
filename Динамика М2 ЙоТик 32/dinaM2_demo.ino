#include <Wire.h>  //библиотека для I2C интерфейса

#include <MGB_MDYN2.h>                                          // библиотека для моторной платы
Adafruit_PWMServoDriver mdyn2 = Adafruit_PWMServoDriver(0x79);  // адрес платы
float power = 0.3;
int kF = 0;

#include <VL53L0X.h>  // библиотека для датчика MGS-D20
VL53L0X lox1;
VL53L0X lox2;

/*
  I2C порт 0x07 - выводы GP16 (SDA), GP17 (SCL)
  I2C порт 0x06 - выводы GP4 (SDA), GP13 (SCL)
  I2C порт 0x05 - выводы GP14 (SDA), GP15 (SCL)
  I2C порт 0x04 - выводы GP5 (SDA), GP23 (SCL)
  I2C порт 0x03 - выводы GP18 (SDA), GP19 (SCL)
*/
#include <MGB_I2C63.h>
// false - для PW548A, true - для PCA9547
MGB_I2C63 mgb_i2c63 = MGB_I2C63(true);
#define GYRO 0x07
#define DIST1 0x05
#define DIST2 0x03
#define BUZ 0x06

#include <MGB_BUZ1.h>  // библиотека для MGB-BUZ1
Adafruit_MCP4725 buzzer;

IPAddress local_IP(192, 168, 4, 7);
// Задаем IP-адрес сетевого шлюза:
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

#include <Arduino.h>
#include <GyverHub.h>
//GyverHub hub;
GyverHub hub("MyDevices", "DynamikaM2", "");  // можно настроить тут, но без F-строк!
gh::Pos posi;
gh::Color colorRGB(0, 0, 0);
int dist1, dist2;

#include <EEPROM.h>
#define EEPROM_SIZE 100
char* AP_SSID;
int number = 0;

void setColorRGB() {
  for (int i = 1; i < 5; i++) {
    mdyn2.rgb_set(i, colorRGB.r, colorRGB.g, colorRGB.b);
  }
}

// билдер
void build(gh::Builder& b) {
  {
    gh::Row r(b);
    b.Color(&colorRGB).label("RGB-светодиоды").size(2).attach(setColorRGB);

    if (b.Button().label("Генератор звука").click()) {
      mgb_i2c63.setBusChannel(BUZ);
      buzzer.note(3, 450);
      buzzer.setVoltage(0, false);
    }

    b.Label_("Dist1").label("Дистанция 1");
    b.Label_("Dist2").label("Дистанция 2");
  }
  // b.Joystick(&posi, 0, 0).label("Управление движением");
  b.Dpad(&posi).label("Управление движением");
  b.Slider_("sld", &power).label("Мощность моторов").range(0, 1, 0.1, 1);
  b.Slider_("sld2", &kF).label("Баланс моторов").range(-10, 10, 1);
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  checkNamber();
  // Инициализация драйвера
  Wire.begin();
  mdyn2.begin();
  // Частота (Гц)
  mdyn2.setPWMFreq(100);
  // Все порты моторной платы выключены
  for (int channel = 0; channel < 16; channel++) {
    mdyn2.setPWM(channel, 0, 0);
  }

  // запуск генератора звука
  mgb_i2c63.setBusChannel(BUZ);
  buzzer.begin(0x60);           // Без перемычки адрес будет 0x61
  buzzer.setVoltage(0, false);  // выключение звука
  buzzer.volume(800);           // громкость (1-999)

  // запуск датчиков расстояния
  mgb_i2c63.setBusChannel(DIST1);
  lox1.init();
  lox1.setTimeout(500);
  lox1.setMeasurementTimingBudget(20000);
  mgb_i2c63.setBusChannel(DIST2);
  lox2.init();
  lox2.setTimeout(500);
  lox2.setMeasurementTimingBudget(20000);

#ifdef GH_ESP_BUILD
  WiFi.mode(WIFI_AP);
  Serial.print("Настоить статический IP: ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Удалось" : "Не удалось!");
  WiFi.softAP(AP_SSID);
  Serial.println(WiFi.softAPIP());
#endif
  // подключить билдер
  hub.onBuild(build);
  // запуск!
  hub.begin();
}

void loop() {
  hub.tick();

  Serial.print("x: ");
  Serial.print(posi.y);
  Serial.print(" y: ");
  Serial.println(posi.x);
  Serial.print("x: ");

  /////////// с джойстика ///////////////
  /*
  static float mA, mB;
  mA = posi.x - posi.y;
  mB = posi.x + posi.y;
  mA = constrain(mA, -100, 100);
  mB = constrain(mB, -100, 100);

  mdyn2.motor_setpower(1, power * mA, false);
  mdyn2.motor_setpower(2, power * mB, false);
*/
  ///////////////////////////////////////////////
  /////////// с крестовины //////////////////////
  if (posi.y == -1) {
    mdyn2.motor_setpower(1, power * 50 + kF, true);
    mdyn2.motor_setpower(2, power * 50 - kF, false);
  } else if (posi.y == 1) {
    mdyn2.motor_setpower(1, power * 50 + kF, false);
    mdyn2.motor_setpower(2, power * 50 - kF, true);
  } else if (posi.x == -1) {
    mdyn2.motor_setpower(1, power * 50, false);
    mdyn2.motor_setpower(2, power * 50, false);
  } else if (posi.x == 1) {
    mdyn2.motor_setpower(1, power * 50, true);
    mdyn2.motor_setpower(2, power * 50, true);
  } else {
    mdyn2.motor_setpower(1, 0, false);
    mdyn2.motor_setpower(2, 0, false);
  }
  /////////////////////////////////////////////////////

  static gh::Timer tmr(1000);  // период 1 секунда

  // каждую секунду будем обновлять датчики
  if (tmr) {
    mgb_i2c63.setBusChannel(DIST1);
    dist1 = lox1.readRangeSingleMillimeters();  // снимаем данные с датчика расстояния
    mgb_i2c63.setBusChannel(DIST2);
    dist2 = lox2.readRangeSingleMillimeters();
    hub.update("Dist1").value(dist1);
    hub.update("Dist2").value(dist2);
  }
}

void checkNamber() {
  EEPROM.get(95, number);
  Serial.println(number);
  if (number == -1) {
    number = random(10000);
    EEPROM.put(95, number);
    EEPROM.commit();
  }

  String n = "DynamicaM2_№-";
  n = n + String(number);

  AP_SSID = strcpy(new char[n.length() + 1], n.c_str());
}
