// подключить библиотеку файловой системы (до #include GyverPortal)
#include <LittleFS.h>

#include <GyverPortal.h>
GyverPortal ui(&LittleFS);  // передать ссылку на fs (SPIFFS/LittleFS)

// Задаем статический IP-адрес:
IPAddress local_IP(192, 168, 4, 4);
// Задаем IP-адрес сетевого шлюза:
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);

#include <EEPROM.h>
#define EEPROM_SIZE 100

char* AP_SSID;
int number = 0;
bool flagLED;

// конструктор страницы
void build() {
  GP.BUILD_BEGIN();
  GP.THEME(GP_DARK);

  GP.TITLE("MGBot контроллер 'ЙОТИК 32' приветстует!");
  GP.HR();

  GP.LABEL("Выберите файл .bin");

  GP.OTA_FIRMWARE("Загрузить");

  M_SPOILER(
    "Ссылка на bin файлы (прошивки)",
    M_BLOCK(
      GP.TEXT("", "", "https://github.com/MAKblC/bin-files");
      GP.BREAK();
      GP.LABEL_BLOCK("Напоминание");
      GP.BREAK();
      GP.TEXT("", "", "Смените WiFi для скачивания!");););

  GP.BUILD_END();
}

void setup() {
  startup();

  if (!LittleFS.begin()) Serial.println("FS Error");

  ui.attachBuild(build);
  ui.attach(action);
  ui.start();
  ui.enableOTA();

  pinMode(4, OUTPUT);
  pinMode(18, OUTPUT);
}

void action() {
}

void loop() {
  static unsigned long timer;

  ui.tick();


  if (millis() - timer >= 500) {
    flagLED = !flagLED;
    digitalWrite(4, flagLED);
    digitalWrite(18, flagLED);
    timer = millis();
  }
}

void startup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  //EEPROM.put(95, number);  //Раскоментить для смены имени сети каждую перезагрузку
  //EEPROM.commit();
  checkNamber();
  WiFi.mode(WIFI_AP);
  Serial.print("Устанавливаем статический IP ---> ");
  Serial.println(WiFi.softAPConfig(local_IP, gateway, subnet) ? "Ready" : "Failed!");
  WiFi.softAP(AP_SSID, "");
  Serial.print("WiFi : ");
  Serial.println(AP_SSID);
  Serial.println(WiFi.softAPIP());
}

void checkNamber() {
  EEPROM.get(95, number);
  Serial.println(number);
  if (number == -1) {
    number = random(10000);
    EEPROM.put(95, number);
    EEPROM.commit();
  }

  String n = "IoTik32_№-";
  n = n + String(number);

  AP_SSID = strcpy(new char[n.length() + 1], n.c_str());
}
