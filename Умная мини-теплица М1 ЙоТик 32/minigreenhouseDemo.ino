// логин и пароль Wi_Fi соединения
#define AP_SSID "GreenHouse_mini_#"
#define AP_PASS ""

#include <Arduino.h>
#include <GyverHub.h>
GyverHub hub;

IPAddress local_IP(192, 168, 4, 4);
// Задаем IP-адрес сетевого шлюза:
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 0, 0);

#define I2C_HUB_ADDR 0x70  // настройки I2C для платы MGB-I2C63
#define EN_MASK 0x08
#define DEF_CHANNEL 0x00
#define MAX_CHANNEL 0x08
#include <Wire.h>
bool TCA9548A;

#define pump 13  // пин насоса
#define led 4    //  пин светодиодной ленты

#include <BH1750.h>  // библиотека датчика освещенности
BH1750 lightMeter;

#include <Adafruit_Sensor.h>  // библиотека датчика температуры, влажности и давления
#include <Adafruit_BME280.h>
Adafruit_BME280 bme280;

#define SOIL_MOISTURE 34  // пины датчика температуры и влажности почвы
#define SOIL_TEMPERATURE 35
// откалиброванные значения
const float air_value = 1587.0;
const float water_value = 800.0;
const float moisture_0 = 0.0;
const float moisture_100 = 100.0;

// Переменные для GP
bool ledStat = 0;
bool pumpStat = 0;
int temp = 0;
int hum = 0;
int light = 0;
int humA = 0;
int tempA = 0;
int press = 0;

// билдер
void build(gh::Builder& b) {
  b.Title("Датчики");
  {
    gh::Row r(b);
    b.Label_("Temp").label("Температура");
    b.Label_("Hum").label("Влажность");
    b.Label_("Pres").label("Давление");
  }
  {
    gh::Row r(b);
    b.Label_("Osv").label("Освещенность");
    b.Label_("HumSoil").label("Влажность почвы");
    b.Label_("TempSoil").label("Температура почвы");
  }
  b.Title("Управление");
  {
    gh::Row r(b);
    if (b.Switch(&ledStat).label("Светодиодная лента").click()) {
      Serial.print("Лента: ");
      Serial.println(ledStat);
      digitalWrite(led, ledStat);
    }
    if (b.Switch(&pumpStat).label("Насос").click()) {
      Serial.print("Насос: ");
      Serial.println(pumpStat);
      digitalWrite(pump, pumpStat);
    }
  }
}

void setup() {
  Wire.begin();
  setI2Cmodule();
  pinMode(pump, OUTPUT);
  pinMode(led, OUTPUT);     // настройка пинов насоса и вентилятора на выход // pump and cooler pins configured on output mode
  digitalWrite(pump, LOW);  // устанавливаем насос и вентилятор изначально выключенными // turn cooler and pump off
  digitalWrite(led, LOW);

  Serial.begin(115200);
  delay(512);

  // Инициируем WiFi
  WiFi.mode(WIFI_AP);
  Serial.print("Настоить статический IP: ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Удалось" : "Не удалось!");
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("IP для подключения: ");
  Serial.println(WiFi.softAPIP());

  lightMeter.begin();  // запуск датчика освещенности

  setBusChannel(0x07);
  bool bme_status = bme280.begin();
  if (!bme_status) {
    Serial.println("Не найден по адресу 0х77, пробую другой...");
    bme_status = bme280.begin(0x76);
    if (!bme_status)
      Serial.println("Датчик не найден, проверьте соединение");
  }
  // указать префикс сети, имя устройства и иконку
  hub.config(F("MyDevices"), F("IoTik32_GH_mini_M1"), F(""));

  // подключить билдер
  hub.onBuild(build);

  // запуск!
  hub.begin();
}

void loop() {
  // =================== ТИКЕР ===================
  // вызываем тикер в главном цикле программы
  // он обеспечивает работу связи, таймаутов и прочего
  hub.tick();

  // =========== ОБНОВЛЕНИЯ ПО ТАЙМЕРУ ===========
  // в библиотеке предусмотрен удобный класс асинхронного таймера
  static gh::Timer tmr(500);  // период 1 секунда

  // каждую секунду будем обновлять заголовок
  if (tmr) {
    light = lightMeter.readLightLevel();
    // Вывод измеренных значений в терминал
    Serial.println(String(light) + " lx");
    setBusChannel(0x07);
    tempA = bme280.readTemperature();
    Serial.println("Air temperature = " + String(tempA) + " *C");
    humA = bme280.readHumidity();
    Serial.println("Air humidity = " + String(humA) + " %");
    press = bme280.readPressure() / 133.3F;
    Serial.print("Air pressure = ");
    Serial.println(String(press) + " mmHg");

    float adc0 = analogRead(SOIL_MOISTURE);
    float adc1 = analogRead(SOIL_TEMPERATURE);
    float temp = ((adc1 / 4095.0 * 6.27) - 0.5) * 100.0;
    float hum = map(adc0, air_value, water_value, moisture_0, moisture_100);

    hub.update(F("Temp")).value(tempA);
    hub.update(F("Hum")).value(humA);
    hub.update(F("Pres")).value(press);
    hub.update(F("Osv")).value(light);
    hub.update(F("HumSoil")).value(hum);
    hub.update(F("TempSoil")).value(temp);
  }
}

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