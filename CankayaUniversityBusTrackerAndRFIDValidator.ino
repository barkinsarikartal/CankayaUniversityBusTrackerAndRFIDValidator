/*
  This code has been developed for use in a module that works both as RFID validator and live bus tracker for Çankaya University.
  This module performs advanced tasks for verifying the eligibility of an RFID card holder and sending real-time location information to the university's servers from the bus it is attached to.
  The module has been developed based on the three previous RFID card reader systems and enhanced with both SIM7600 SIM Card Module and WiFi.
  Previous systems: https://github.com/barkinsarikartal/DoorLockSystemWithRFID
                    https://github.com/barkinsarikartal/CankayaUniversityDoorLockSystemWithWiFi
                    https://github.com/barkinsarikartal/CankayaUniversityRFIDValidationModuleWithWiFi

  The code in operation performs the following statements:
  1) Read RFID cards by RC522 and check their eligibility,
  2) Control if WiFi or SIM can be used,
  3) Post data of the read RFID card IDs and the timestamps of when they were read by the RC522,
  4) Post location data of the school bus to university's servers,
  5) Check updates on JSON file.

  This program works on four different scenarios by checking the current situation:
  1) If both WiFi and SIM Card Module is available, the program will prioritize WiFi to save cellular data.
  2) If only WiFi is available, the program will run the entire process over WiFi.
  3) If only SIM Card Module is available, the program will run the entire process over SIM Card Module.
  4) If neither is available, the program will run locally.

  This code accounts for 1121881 bytes (33%) of program storage space and 40576 bytes (12%) of dynamic memory.
  (These statistics are for the ESP32-S3-DevKitC-1 N16R8 with "8M with spiffs (3MB APP/1.5MB SPIFFS)" partition scheme option selected.)

  Contributors:
    Barkın SARIKARTAL     Computer Engineering and Electrical and Electronics Student, Çankaya University     sarikartalbarkin@gmail.com
  Supporters:
    Abdül Kadir GÖRÜR     Dr. Lecturer, Computer Engineering Department, Çankaya University                   agorur@cankaya.edu.tr
    Ayhan ARICI           Engineer, IT Department, Çankaya University                                         ayhanarici@cankaya.edu.tr
    Burçin TUNA           Head of Department, IT Department, Çankaya University                               btuna@cankaya.edu.tr
    H. Hakan MARAŞ        Rector, Çankaya University                                                          hhmaras@cankaya.edu.tr

    Last Edited: 28.09.2024.
*/

//definition of SIM Card Module version and digital pins used for modules
#define TINY_GSM_MODEM_SIM7600  //Using SIM7600E
#define TINY_GSM_YIELD_MS 2
#define MODEM_AVAILABLE 41      //SIM Card Module's 3v3 pin is connected to GPIO41
#define GPS_AVAILABLE 7         //GPS on-off switch is connected between GPIO7 and GND
#define SerialAT Serial1        //SIM Card Module is connected to ESP32 with Serial1 channel
#define ARR_SIZE 15000          //Array size is 15000 in case of there are more cards on the JSON file
#define QUEUE_SIZE 300          //Queue between tasks can hold up to 300 elements
#define EEPROM_SIZE 64          //Starting EEPROM with 64 bytes
#define NAME_LENGTH 21          //Every card holder has 20 character name + null character
#define BUZZER_PIN 18           //Buzzer is connected between GPIO18 and GND
#define MODEM_TX 16             //SIM Card Module's RXD pin is connected to GPIO16
#define MODEM_RX 15             //SIM Card Module's TXD pin is connected to GPIO15
#define NUM_LEDS 8              //WS2812 (5050) has 8 LEDs
#define LED_PIN 17              //WS2812's IN pin is connected to GPIO17
#define RST_PIN 0               //RC522's RST pin is connected to GPIO0

//used libraries
#include <SPI.h>                      //This library is automatically installed when ESP32 add-on is installed in the Arduino IDE.
#include <Wire.h>                     //This library is automatically installed when ESP32 add-on is installed in the Arduino IDE.
#include <time.h>                     //This library is automatically installed when ESP32 add-on is installed in the Arduino IDE.
#include <WiFi.h>                     //This library is automatically installed when ESP32 add-on is installed in the Arduino IDE.
#include <RTClib.h>                   //Using version: 2.1.4
#include <EEPROM.h>                   //This library is automatically installed when ESP32 add-on is installed in the Arduino IDE.
#include <SPIFFS.h>                   //This library is automatically installed when ESP32 add-on is installed in the Arduino IDE.
#include <Arduino.h>                  //Comes with Arduino IDE
#include <WiFiUdp.h>                  //This library is automatically installed when ESP32 add-on is installed in the Arduino IDE.
#include <MFRC522v2.h>                //Using version: 2.0.4
#include <NTPClient.h>                //Using version: 3.2.1
#include <SSLClient.h>                //Using version: 1.3.2
#include <HTTPClient.h>               //This library is automatically installed when ESP32 add-on is installed in the Arduino IDE.
#include <ArduinoJson.h>              //Using version: 7.0.4
#include <JsonListener.h>             //Comes with JsonStreamingParser
#include <MFRC522Debug.h>             //Comes with MFRC522v2
#include <TinyGsmClient.h>            //Using version: 0.12.0
#include <SoftwareSerial.h>           //Comes with Arduino IDE
#include <MFRC522DriverSPI.h>         //Comes with MFRC522v2
#include <ArduinoHttpClient.h>        //Using version: 0.6.1
#include <LiquidCrystal_I2C.h>        //Using version: 1.1.3
#include <Adafruit_NeoPixel.h>        //Using version: 1.12.3
#include <JsonStreamingParser.h>      //Using version: 2.1.0
#include <MFRC522DriverPinSimple.h>   //Comes with MFRC522v2
#include "esp_heap_caps.h"            //This library is automatically installed when ESP32 add-on is installed in the Arduino IDE.
#include "esp_log.h"                  //This library is automatically installed when ESP32 add-on is installed in the Arduino IDE.

const char apn[]  = "internet";
const char user[] = "";
const char pass[] = "";
const char* ssid = "SSID";
const char* password = "PASSWORD";
const char* hashFileName = "/hash.txt";
const char* jsonFileName1 = "/large1.json";
const char* jsonFileName2 = "/large2.json";
const char* notSentCardsFileName = "/notSentCardsFile.txt";
const char* ntpServer1 = "pool.ntp.org";
const char* ntpServer2 = "worldtimeapi.org";
const char* ntpServer2Path = "/api/timezone/Europe/Istanbul";
const char* bearerToken = "bearer-token";
const char* hashAddress = "http-address-to-hash";
const char* getAddress = "http-address-to-json-file";
const char* postAddress = "http-address-to-post-read-cards";
const char* mainServer = "server-address";
const char* postAddressPath = "path-to-post-read-cards";
const char* hashAddressPath = "path-to-hash";
const char* getAddressPath = "path-to-json-file";
const char* GPSPostAddress = "path-to-post-GPS-location";

int valueRead;
int personCount;
int simstatus = 0;
int GPSPinState = 0;
int eepromAddress = 0;
int modemPinState = 0;

String imei = "imei-of-SIM-module";
String newHash = "";

unsigned long lastCardReadTime = 0;
unsigned long lastPostTry = 0;
unsigned long lastSIMTry = 0;
unsigned long lastWiFiTry = 0;
unsigned long lastGPSPost = 0;
unsigned long lastMenuPrint = 0;
unsigned long GPSTimer = 10000;
unsigned long postTryTimer = 45000;
unsigned long rc522ResetTimer = 180000;
unsigned long SIMTryTimer = 15000;
unsigned long menuPrintTimer = 30000;
const long utcOffsetInSeconds = 10800;

bool update = false;
bool updating = false;
bool doneStrip = false;
bool killedStrip = false;
bool WiFiAv = false;
bool SIMAv = false;
bool GPSAv = false;
bool NoSIMPlugged = false;
bool NoGPSPlugged = true;
bool finishedPosting = true;
bool initialRTCUpdate = false;
bool initialSIMbegin = false;
bool initialWiFibegin = false;
bool initialCardsUpdate = false;

byte networkSign[] = { //Wi-Fi available logo
  B01110,
  B11011,
  B10001,
  B00100,
  B01010,
  B00000,
  B00100,
  B00000,
};

byte mobileLogo1[] = { //low quality cellular data logo
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B01000,
  B11000,
  B11000,
};

byte mobileLogo2[] = { //mid quality cellular data logo
  B00000,
  B00000,
  B00000,
  B00010,
  B00110,
  B01110,
  B11110,
  B11110,
};

byte mobileLogo3[] = { //high quality cellular data logo
  B00000,
  B00000,
  B00001,
  B00011,
  B00111,
  B01111,
  B11111,
  B11111,
};

byte GPSLogo[] = { //receiving GPS location from sastellites logo
  B01110,
  B11111,
  B10001,
  B10101,
  B10001,
  B01010,
  B00100,
  B00000,
};

byte noSIMLogo[] = { //cellular data is not available logo
  B00101,
  B00010,
  B00101,
  B00000,
  B11000,
  B11100,
  B11100,
  B11100,
};

byte noWiFiLogo[] = { //Wi-Fi is not available logo
  B00000,
  B01010,
  B00100,
  B01010,
  B00000,
  B00000,
  B00100,
  B00000,
};

struct Person { //struct of each card holder
  byte cardID[4];
  char name[NAME_LENGTH];
};

struct ReadCard { //struct of read card to send to queue between tasks
  String cardID;
  String readTime;
  String validation;
};

//creation of objects from libraries' classes
TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
SSLClient secure_layer(&client);
LiquidCrystal_I2C lcd(0x27,20,4);
RTC_DS3231 rtc;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
MFRC522DriverPinSimple ss_pin(10);
MFRC522DriverSPI driver{ss_pin};
MFRC522 reader{driver};
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, ntpServer1, utcOffsetInSeconds);

//FreeRTOS definitions and dynamic personArray initialization
Person* personArray = nullptr;
void StripLoadingTask(void *parameter);
TaskHandle_t stripLoadingTaskHandle = NULL;
QueueHandle_t cardQueue;
SemaphoreHandle_t xNotSentCardsMutex;

class MyJsonListener : public JsonListener { //Inheriting MyJsonListener from JsonListener library that parses JSON.
public:
  void whitespace(char c) override {}
  void startDocument() override {}
  void key(String key) override {
    currentKey = key;
  }
  void value(String value) override {
    if (currentKey == "CardId") {
      currentCardId = value;
    }
    else if (currentKey == "Name") {
      currentName = value;
    }
  }
  void endArray() override {}
  void endObject() override {
    objectCount++;
    currentCardId.trim();
    currentCardId.toUpperCase();
    fillTheArray();
  }
  void endDocument() override {
    personCount = objectCount;
    Serial.print("Kayıtlı kişi sayısı: ");
    Serial.println(personCount);
    fillRestofArray(objectCount);
  }
  void startArray() override {}
  void startObject() override {
    if (String("CardId") == currentKey) {
      currentCardId = "";
    }
    else if (String("Name") == currentKey) {
      currentName = "";
    }
  }
private:
  String currentKey;
  String currentCardId;
  String currentName;
  int objectCount = 0;

  void fillTheArray() { //To fill the next array element after each object is parsed
    static int elementIndex = 0;
    while (elementIndex < ARR_SIZE) {
      Person &person = personArray[elementIndex];
      if (currentCardId.length() != 8 || currentName.isEmpty()) {
        for (int i = elementIndex; i < ARR_SIZE; ++i) {
          personArray[i].cardID[0] = 0xFF;
          personArray[i].cardID[1] = 0xFF;
          personArray[i].cardID[2] = 0xFF;
          personArray[i].cardID[3] = 0xFF;
          strncpy(personArray[i].name, "AAAAAAAA", NAME_LENGTH);
          personArray[i].name[NAME_LENGTH - 1] = '\0';
        }
        for (int k = 0; k < ARR_SIZE; ++k) {
          personArray[k].cardID[0] = 0xFF;
          personArray[k].cardID[1] = 0xFF;
          personArray[k].cardID[2] = 0xFF;
          personArray[k].cardID[3] = 0xFF;
          strncpy(personArray[k].name, "AAAAAAAA", NAME_LENGTH);
          personArray[k].name[NAME_LENGTH - 1] = '\0';
        }
      }
      else {
        for (int i = 0; i < 4; ++i) {
          person.cardID[i] = strtol(currentCardId.substring(i * 2, i * 2 + 2).c_str(), nullptr, 16);
        }
        strncpy(person.name, currentName.c_str(), NAME_LENGTH);
        person.name[NAME_LENGTH - 1] = '\0';
        elementIndex++;
      }
      break;
    }
  }

  void fillRestofArray(int objCnt) { //Function to fill the rest of array with empty elements
    if (objCnt < ARR_SIZE) {
      for (int i = objCnt; i < ARR_SIZE; ++i) {
        personArray[i].cardID[0] = 0xFF;
        personArray[i].cardID[1] = 0xFF;
        personArray[i].cardID[2] = 0xFF;
        personArray[i].cardID[3] = 0xFF;
        strncpy(personArray[i].name, "AAAAAAAA", NAME_LENGTH);
        personArray[i].name[NAME_LENGTH - 1] = '\0';
      }
    }
  }
};

void setup() { //Setup function to start every module used and allocate and fill personArray
  SPI.begin();
  delay(10);
  Wire.begin();
  delay(10);
  Serial.begin(115200);
  delay(10);
  esp_log_level_set("wifi", ESP_LOG_NONE);
  delay(10);

  strip.begin();
  delay(10);
  strip.show();

  pinMode(MODEM_AVAILABLE, INPUT);
  pinMode(GPS_AVAILABLE, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  xTaskCreatePinnedToCore( //Strip Loading Task
    StripLoadingTask,
    "Strip Loading Task",
    4096,
    NULL,
    1,
    NULL,
    0
  );

  if (!rtc.begin()) {
    Serial.println("RTC modulu bulunamadi.");
    ESP.restart();
  }

  lcd.init();
  delay(10);
  lcd.backlight();
  lcd.clear();
  lcd.createChar(0, networkSign);
  lcd.createChar(1, GPSLogo);
  lcd.createChar(2, mobileLogo1);
  lcd.createChar(3, mobileLogo2);
  lcd.createChar(4, mobileLogo3);
  lcd.createChar(5, noSIMLogo);
  lcd.createChar(6, noWiFiLogo);
  lcd.noBlink();
  LCDInfo();

  digitalWrite(BUZZER_PIN, HIGH);
  delay(200);
  digitalWrite(BUZZER_PIN, LOW);

  DateTime now = rtc.now();
  Serial.print("Tarih: ");
  Serial.print(now.day(), DEC); Serial.print('/'); Serial.print(now.month(), DEC); Serial.print('/'); Serial.print(now.year(), DEC);
  Serial.print(", Saat: ");
  Serial.print(now.hour(), DEC); Serial.print(':'); Serial.print(now.minute(), DEC); Serial.print(':'); Serial.println(now.second(), DEC);

  PrintLogos(simstatus, WiFiAv, GPSAv);

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS baslatilamadi.");
    ESP.restart();
  }

  if (!EEPROM.begin(EEPROM_SIZE)) {
    Serial.println("EEPROM baslatma basarisiz.");
    ESP.restart();
  }

  EEPROM.get(eepromAddress, valueRead);

  cardQueue = xQueueCreate(QUEUE_SIZE, sizeof(ReadCard));
  AllocateArrays();
  ParseJsonFromSPIFFS();

  if(!reader.PCD_Init()){
    Serial.println("RC522 bulunamadi.");
    delay(500);
    ESP.restart();
  }
  delay(10);

  reader.PICC_HaltA();
  reader.PCD_StopCrypto1();

  modemPinState = digitalRead(MODEM_AVAILABLE);
  GPSPinState = digitalRead(GPS_AVAILABLE);
  Serial.print("SIM Pin State: "); Serial.println(modemPinState);
  Serial.print("GPS Pin State: "); Serial.println(GPSPinState);
  LcdMenu();

  doneStrip = true;
  while (!killedStrip) {
    delay(10);
  }
  strip.clear();
  strip.show();

  xNotSentCardsMutex = xSemaphoreCreateMutex();
  if (xNotSentCardsMutex == NULL) {
    Serial.println("Mutex olusturulamadi.");
    delay(500);
    ESP.restart();
  }
  else {
    xTaskCreatePinnedToCore( //Core 0 Task
      Core0Task,
      "Core0Task",
      16384,
      NULL,
      1,
      NULL,
      0
    );
    xTaskCreatePinnedToCore( //Core 1 Task
      Core1Task,
      "Core1Task",
      16384,
      NULL,
      1,
      NULL,
      1
    );
  }
}

void loop() { //Empty loop because this code uses both of the cores with FreeRTOS
  //empty loop
}

// <<<<<<<<<<<<<<<<<<<< FreeRTOS Functions >>>>>>>>>>>>>>>>>>>> //

void Core0Task(void* parameter) { //Core 0 Task
  Serial.println("Core0Task has begun.");
  ReadCard queueCard;
  while (1) {
    modemPinState = digitalRead(MODEM_AVAILABLE);
    GPSPinState = digitalRead(GPS_AVAILABLE);
    if (modemPinState == HIGH) { //To update boolians about SIM Card Module
      NoSIMPlugged = false;
      if(GPSPinState == HIGH){
        NoGPSPlugged = false;
      }
      else {
        NoGPSPlugged = true;
        GPSAv = false;
      }
    }
    else {
      NoSIMPlugged = true;
      SIMAv = false;
      simstatus = 0;
      NoGPSPlugged = true;
      GPSAv = false;
    }
    if (reader.PICC_IsNewCardPresent() && reader.PICC_ReadCardSerial()) { //To check read card's eligibility
      lastCardReadTime = millis();
      if (reader.uid.size == 4) {
        byte readCardID[4];
        for (int i = 0; i < 4; ++i) {
          readCardID[i] = reader.uid.uidByte[i];
        }
        ReadCard newCard;
        String name;
        DateTime now = rtc.now();
        newCard.readTime = String(now.unixtime());
        newCard.cardID = GetCardID();
        Serial.println("--------------------"); Serial.println("READCARD FUNCTION:");
        if (BinarySearch(personArray, personCount, readCardID, name) != -1) {
          Serial.println("tanimli kart.");
          newCard.validation = "1";
          LcdValidScreen(name);
          vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        else {
          Serial.println("gecersiz kart.");
          newCard.validation = "0";
          LcdDeclinedScreen();
        }
        Serial.println("Queue'ye gonderilen kart bilgisi: " + newCard.cardID + " " + newCard.readTime + " " + newCard.validation);
        xQueueSend(cardQueue, &newCard, pdMS_TO_TICKS(100));
        UBaseType_t numberOfItems = uxQueueMessagesWaiting(cardQueue);
        Serial.print("Kuyruktaki eleman sayisi: "); Serial.println(numberOfItems);
        Serial.println("--------------------");
        LcdMenu();
        reader.PICC_HaltA();
        reader.PCD_StopCrypto1();
      }
    }
    if (millis() - lastCardReadTime > rc522ResetTimer) { //To reset RC522 if it didn't read any cards in a while
      if(ResetRC522()){
        vTaskDelay(100 / portTICK_PERIOD_MS);
        Serial.println("RC522 yeniden baslatildi.");
        lastCardReadTime = millis();
      }
    }
    if (initialWiFibegin) { //To check and reconnect to Wi-Fi
      if (WiFiStatusControl()) {
        WiFiAv = true;
      }
      else {
        WiFiAv = false;
        WiFi.reconnect();
      }
    }
    if (xSemaphoreTake(xNotSentCardsMutex, pdMS_TO_TICKS(100)) == pdTRUE) { //To write read cards to the memory
      if (xQueueReceive(cardQueue, &queueCard, pdMS_TO_TICKS(50))) {
        File notSentCardsFile = SPIFFS.open(notSentCardsFileName, FILE_APPEND);
        Serial.println("--------------------"); Serial.println("CORE 1 QUEUE FUNCTION:");
        if (!notSentCardsFile) {
          Serial.println("notSentCardsFile ekleme icin acilamadi");
        }
        else {
          if (queueCard.cardID.length() == 8 && queueCard.readTime.length() == 10) {
            String notSentCardData = queueCard.cardID + queueCard.readTime + queueCard.validation;
            if (notSentCardsFile.print(notSentCardData)) {
              Serial.println(notSentCardData + " dosyaya başarıyla yazildi.");
            }
            else {
              Serial.println(notSentCardData + " dosyaya yazilamadi.");
            }
          }
          else {
            Serial.println("Kart ID 8 karakter veya unixtime 10 basamakli olmadigi icin veri hafizaya yazilmadi.");
          }
          notSentCardsFile.close();
        }
        Serial.println("--------------------");
      }
      xSemaphoreGive(xNotSentCardsMutex);
    }
    if (millis() - lastMenuPrint > menuPrintTimer) { //To refresh LCD Screen in a while
      LcdMenu();
      lastMenuPrint = millis();
    }
    PrintLogos(simstatus, WiFiAv, GPSAv);
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

void Core1Task(void* parameter) { //Core 1 Task
  Serial.println("Core1Task has begun.");
  ReadCard newCard;
  size_t readSize;
  while (1) {
    if (!initialWiFibegin) { //To start Wi-Fi connection
      WiFi.disconnect(true);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      WiFi.begin(ssid, password);
      WiFi.setAutoReconnect(true);
      unsigned long wifiStartTime = millis();
      while (millis() - wifiStartTime < 10000) {
        if (WiFi.isConnected()) {
          WiFiAv = true;
          break;
        }
      }
      initialWiFibegin = true;
    }
    if (!initialSIMbegin) { //To start SIM Card Module
      if (modemPinState == HIGH) {
        //Serial.println("modemPinState HIGH");
        SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if(modem.restart()){
          if(modem.waitForNetwork()){
            if(modem.gprsConnect(apn, user, pass)) {
              NoSIMPlugged = false;
              SIMAv = true;
              int signalQuality = modem.getSignalQuality();
              if(signalQuality > 31 || signalQuality < 2){
                simstatus = 0;
              }
              else if(signalQuality >= 2 && signalQuality < 10){
                simstatus = 1;
              }
              else if(signalQuality >= 10 && signalQuality <= 20){
                simstatus = 2;
              }
              else{
                simstatus = 3;
              }
              if (GPSPinState == HIGH) {
                NoGPSPlugged = false;
                if(modem.enableGPS()){
                  vTaskDelay(1000 / portTICK_PERIOD_MS);
                  float lat, lon;
                  if(modem.getGPS(&lat, &lon)) {
                    Serial.println("location: " + String(lat,8) + ", " + String(lon,8));
                    GPSAv = true;
                  }
                }
              }
            }
            else{
              Serial.println("gprs baglanamadi");
            }
          }
          else{
            Serial.println("network beklenemedi");
          }
        }
        else{
          Serial.println("restart atilamadi");
        }
      }
      else {
        SIMAv = false;
        GPSAv = false;
        NoSIMPlugged = true;
        NoGPSPlugged = true;
      }
      initialSIMbegin = true;
    }
    if (!initialRTCUpdate) { //To update RTC (BIOS Time)
      Serial.println("--------------------");
      if (WiFi.isConnected() && !SIMAv) {
        Serial.println("INITIAL WiFi RTC FUNCTION:");
        UpdateRTCFromNTPWiFi();
      }
      else if (SIMAv) {
        Serial.println("INITIAL SIM RTC FUNCTION:");
        UpdateRTCFromNTPSIM();
        initialRTCUpdate = true;
      }
      initialRTCUpdate = true;
      Serial.println("INITIAL RTC END");
      Serial.println("--------------------");
    }
    if (!NoSIMPlugged) { //To check if SIM Card Module is available. If it is not available, then restarts the module
      if(SIMStatusControl()) {
        SIMAv = true;
        int signalQuality = modem.getSignalQuality();
        if(signalQuality > 31 || signalQuality < 2){
          simstatus = 0;
        }
        else if(signalQuality >= 2 && signalQuality < 10){
          simstatus = 1;
        }
        else if(signalQuality >= 10 && signalQuality <= 20){
          simstatus = 2;
        }
        else{
          simstatus = 3;
        }
        if(!NoGPSPlugged && !GPSAv) {
          if(modem.enableGPS()){
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            float lat, lon;
            if(modem.getGPS(&lat, &lon)) {
              Serial.println("location: " + String(lat,8) + ", " + String(lon,8));
              GPSAv = true;
            }
          }
        }
      }
      else {
        SIMAv = false;
        GPSAv = false;
        simstatus = 0;
        if(millis() - lastSIMTry > SIMTryTimer) {
          SerialAT.end();
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
          vTaskDelay(1000 / portTICK_PERIOD_MS);
          if(modem.restart()){
            if(modem.waitForNetwork()){
              if(modem.gprsConnect(apn, user, pass)) {
                NoSIMPlugged = false;
                SIMAv = true;
                int signalQuality = modem.getSignalQuality();
                if(signalQuality > 31 || signalQuality < 2){
                  simstatus = 0;
                }
                else if(signalQuality >= 2 && signalQuality < 10){
                  simstatus = 1;
                }
                else if(signalQuality >= 10 && signalQuality <= 20){
                  simstatus = 2;
                }
                else{
                  simstatus = 3;
                }
                if (GPSPinState == HIGH) {
                  NoGPSPlugged = false;
                  if(modem.enableGPS()){
                    vTaskDelay(500 / portTICK_PERIOD_MS);
                    float lat, lon;
                    if(modem.getGPS(&lat, &lon)) {
                      Serial.println("location: " + String(lat,8) + ", " + String(lon,8));
                      GPSAv = true;
                    }
                  }
                }
              }
              else{
                Serial.println("gprs baglanamadi");
              }
            }
            else{
              Serial.println("network beklenemedi");
            }
          }
          else{
            Serial.println("restart atilamadi");
          }
          lastSIMTry = millis();
        }
      }
    }
    if (!initialCardsUpdate) { //To check if there is an update on JSON file from server
      unsigned long startUpdateFunction = millis();
      if(WiFi.isConnected()) {
        Serial.println("--------------------");
        Serial.println("WIFI UPDATE FUNCTION:");
        updating = true;
        if (!IsHashEqualWiFi()) {
          FetchAndSaveJsonWiFi();
        }
        UpdateRTCFromNTPWiFi();
        initialCardsUpdate = true;
        updating = false;
        unsigned long endUpdateFunction = millis();
        Serial.print("update fonksiyonunun tamamlanmna suresi (ms): "); Serial.println(endUpdateFunction - startUpdateFunction);
        Serial.println("--------------------");
      }
      else if(!WiFi.isConnected() && SIMAv) {
        Serial.println("--------------------");
        Serial.println("SIM UPDATE FUNCTION:");
        updating = true;
        if (!IsHashEqualSIM()) {
          FetchAndSaveJsonSIM();
        }
        UpdateRTCFromNTPSIM();
        initialCardsUpdate = true;
        updating = false;
        unsigned long endUpdateFunction = millis();
        Serial.print("update fonksiyonunun tamamlanmna suresi (ms): "); Serial.println(endUpdateFunction - startUpdateFunction);
        Serial.println("--------------------");
      }
    }
    if (!NoGPSPlugged && SIMAv) { //To post GPS location to the server
      if (millis() - lastGPSPost > GPSTimer) {
        Serial.println("--------------------"); Serial.println("GPS Post Function:");
        float lat, lon, speed;
        if(modem.getGPS(&lat, &lon, &speed)) {
          GPSAv = true;
          //Serial.println("loc: " + String(lat,8) + " " + String(lon,8));
          DateTime GPSNow = rtc.now();
          uint32_t GPSUnixtime = GPSNow.unixtime();
          String postString = SerializeToGPSJson(lat, lon, speed, GPSUnixtime);
          Serial.println("postGPS: " + postString);
          HttpClient httpPostGPS(secure_layer, mainServer, 443);
          httpPostGPS.beginRequest();
          httpPostGPS.post(GPSPostAddress);
          httpPostGPS.sendHeader("Authorization", String("Bearer ") + bearerToken);
          httpPostGPS.sendHeader("Content-Type", "application/json");
          httpPostGPS.sendHeader("Content-Length", postString.length());
          httpPostGPS.endRequest();
          httpPostGPS.print(postString);
          int httpPostResponseCode = httpPostGPS.responseStatusCode();
          int contentLength = httpPostGPS.contentLength();
          if (httpPostResponseCode == 200 && contentLength > 0) {
            String postResponse = httpPostGPS.responseBody();
            Serial.print("Sunucu yaniti: "); Serial.println(postResponse);
          }
          else {
            Serial.printf("PostGPSData POST istegi basarisiz, hata: %d\n", httpPostResponseCode);
          }
          httpPostGPS.stop();
        }
        else{
          GPSAv = false;
          Serial.println("modemden post icin gps verisi alinamadi.");
        }
        lastGPSPost = millis();
        Serial.println("--------------------");
      }
    }
    if (millis() - lastPostTry > postTryTimer) { //To post cards to the server
      if (xSemaphoreTake(xNotSentCardsMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        if (IsThereNotSentCards(readSize)) {
          if(WiFi.isConnected()) { //to post cards with Wi-Fi
            Serial.println("--------------------"); Serial.println("WIFI POST FUNCTION:");
            unsigned long startPostTryFuntion = millis();
            size_t tempSize = readSize;
            String notSentCardsString;
            notSentCardsString.reserve(readSize);
            finishedPosting = false;
            if (GetNotSentCardsFromMemory(notSentCardsString, readSize)) {
              int tripleCounter = 0;
              while (!finishedPosting && WiFi.isConnected()) {
                HTTPClient httpPost;
                httpPost.begin(postAddress);
                httpPost.addHeader("Authorization", "Bearer " + String(bearerToken));
                httpPost.addHeader("Content-Type", "application/json");
                httpPost.setTimeout(10000);
                Serial.print("kalan kart sayisi: "); Serial.println(tempSize/19);
                //Serial.println("notSentCardsString: " + notSentCardsString);
                String tempData = notSentCardsString.substring(0, 19);
                //Serial.print("tempData: "); Serial.println(tempData);
                notSentCardsString = notSentCardsString.substring(19);
                String postString = SerializeToJson(tempData);
                Serial.println("postString = " + postString);
                int httpPostResponseCode = httpPost.POST(postString);
                String postResponse = httpPost.getString();
                if (httpPostResponseCode == 200) {
                  Serial.print("Sunucu yaniti: "); Serial.println(postResponse);
                  if (postResponse != "\"Card Read Successful\"") {
                    notSentCardsString += tempData;
                  }
                }
                else {
                  Serial.println("hatali sunucu yaniti: " + postResponse);
                  Serial.printf("postNotSentCards POST istegi basarisiz, hata: %s\n", httpPost.errorToString(httpPostResponseCode).c_str());
                  notSentCardsString += tempData;
                }
                tempSize -= 19;
                tripleCounter++;
                if (tempSize <= 0 ||  tripleCounter == 6) {
                  finishedPosting = true;
                  httpPost.end();
                  unsigned long endPostTryFuntion = millis();
                  Serial.print("Fonksiyon tamamlanma suresi (ms): "); Serial.println(endPostTryFuntion - startPostTryFuntion);
                }
              }
              SaveNotSentCardsToMemory(notSentCardsString);
            }
            Serial.println("--------------------");
          }
          else if(!WiFi.isConnected() && SIMAv) { //to post cards with cellular data
            Serial.println("--------------------"); Serial.println("SIM POST FUNCTION:");
            unsigned long startPostTryFuntion = millis();
            size_t tempSize = readSize;
            String notSentCardsString;
            notSentCardsString.reserve(readSize);
            finishedPosting = false;
            if (GetNotSentCardsFromMemory(notSentCardsString, readSize)) {
              int tripleCounter = 0;
              while (!finishedPosting) {
                Serial.print("kalan kart sayisi: "); Serial.println(tempSize/19);
                String tempData = notSentCardsString.substring(0, 19);
                //Serial.print("tempData: "); Serial.println(tempData);
                notSentCardsString = notSentCardsString.substring(19);
                String postString = SerializeToJson(tempData);
                Serial.println("postString = " + postString);
                HttpClient httpPostSIM(secure_layer, mainServer, 443);
                httpPostSIM.beginRequest();
                httpPostSIM.post(postAddressPath);
                httpPostSIM.sendHeader("Authorization", String("Bearer ") + bearerToken);
                httpPostSIM.sendHeader("Content-Type", "application/json");
                httpPostSIM.sendHeader("Content-Length", postString.length());
                httpPostSIM.endRequest();
                httpPostSIM.print(postString);
                int httpPostResponseCode = httpPostSIM.responseStatusCode();
                int contentLength = httpPostSIM.contentLength();
                if (httpPostResponseCode == 200 && contentLength > 0) {
                  String postResponse = httpPostSIM.responseBody();
                  Serial.print("Sunucu yaniti: "); Serial.println(postResponse);
                  if (postResponse != "\"Card Read Successful\"") {
                    notSentCardsString += tempData;
                  }
                }
                else {
                  Serial.printf("postNotSentCards POST istegi basarisiz, hata: %d\n", httpPostResponseCode);
                  notSentCardsString += tempData;
                }
                httpPostSIM.stop(); 
                tempSize -= 19;
                tripleCounter++;
                if (tempSize <= 0 || tripleCounter == 3) {
                  finishedPosting = true;
                  unsigned long endPostTryFuntion = millis();
                  Serial.print("Fonksiyon tamamlanma suresi (ms): "); Serial.println(endPostTryFuntion - startPostTryFuntion);
                }
              }
              SaveNotSentCardsToMemory(notSentCardsString);
            }
            Serial.println("--------------------");
          }
        }
        lastPostTry = millis();
        xSemaphoreGive(xNotSentCardsMutex);
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  vTaskDelete(NULL);
}

// <<<<<<<<<<<<<<<<<<<< Initial Functions >>>>>>>>>>>>>>>>>>>> //

void StripLoadingTask(void *parameter) { //Function to play an animation with WS2812 (5050)
  while (!doneStrip) {
    for (int i = 0; i < NUM_LEDS - 1; i++) {
      strip.clear();
      strip.setPixelColor(i, strip.Color(50, 50, 50)); //white light
      strip.show();
      delay(80);
    }
    for (int i = NUM_LEDS - 1; i > 0; i--) {
      strip.clear();
      strip.setPixelColor(i, strip.Color(50, 50, 50)); //white light
      strip.show();
      delay(80);
    }
    if (doneStrip) {
      break;
    }
  }
  killedStrip = true;
  vTaskDelete(NULL);
}

void AllocateArrays() { //Function to allocate arrays from PSRAM
  personArray = (Person *)heap_caps_malloc(sizeof(Person) * ARR_SIZE, MALLOC_CAP_SPIRAM);
  if (personArray == NULL) {
    Serial.println("PSRAM'den bellek allocate edilemedi.");
    ESP.restart();
  }
}

void ParseJsonFromSPIFFS() { //Function to parse JSON from memory
  const char* filePath = "";
  if (valueRead == 1) {
    filePath = jsonFileName1;
  }
  else {
    filePath = jsonFileName2;
  }
  File file = SPIFFS.open(filePath, FILE_READ);
  if (!file) {
    ESP.restart();
  }
  JsonStreamingParser parser;
  MyJsonListener listener;
  parser.setListener(&listener);

  while (file.available()) {
    char c = file.read();
    parser.parse(c);
  }
  file.close();
}

// <<<<<<<<<<<<<<<<<<<< Core 0 Functions >>>>>>>>>>>>>>>>>>>> //

String GetCardID() { //Function to get read RFID card's ID
  String id = "";
  for (byte i = 0; i < reader.uid.size; i++) {
    if (reader.uid.uidByte[i] < 0x10) {
      id += "0";
    }
    id += String(reader.uid.uidByte[i], HEX);
  }
  id.toUpperCase();
  return id;
}

int BinarySearch(const Person* persons, int size, const byte targetID[4], String &ownerName) { //Function to find if read card ID is valid from personArray
  int left = 0;
  int right = size - 1;
  while (left <= right) {
    int mid = left + (right - left) / 2;
    if (memcmp(persons[mid].cardID, targetID, 4) == 0) {
      ownerName = String(persons[mid].name);
      ownerName.toUpperCase();
      return mid;  //found case
    }
    if (memcmp(persons[mid].cardID, targetID, 4) < 0) {
      left = mid + 1;
    }
    else {
      right = mid - 1;
    }
  }
  return -1; //couldn't find case
}

bool ResetRC522() { //Function to reset RC522
  return reader.PCD_Init();
}

bool WiFiStatusControl() { //Function to check if Wi-Fi is connected
  return WiFi.status() == 3;
}

// <<<<<<<<<<<<<<<<<<<< Core 1 Functions >>>>>>>>>>>>>>>>>>>> //

void UpdateRTCFromNTPWiFi() { //Function to update DS3231's time with the unix time coming from the NTP server.
  timeClient.begin();
  DateTime rtcnow = rtc.now();
  timeClient.update();
  unsigned long ntpEpochTime = timeClient.getEpochTime();
  Serial.print("ntp time: "); Serial.println(ntpEpochTime);
  if (String(ntpEpochTime).length() == 10) {
    DateTime ntpTime = DateTime(ntpEpochTime);
    rtc.adjust(ntpTime);
    Serial.println("RTC Guncellendi.");
  }
  timeClient.end();
}

void UpdateRTCFromNTPSIM() { //Function to update DS3231's time with the unix time coming from SIM Card Module.
  int   ntp_year, ntp_month, ntp_day, ntp_hour, ntp_min, ntp_sec;
  float ntp_timezone;
  if (modem.getNetworkTime(&ntp_year, &ntp_month, &ntp_day, &ntp_hour,
                           &ntp_min, &ntp_sec, &ntp_timezone)) {
    rtc.adjust(DateTime(ntp_year, ntp_month, ntp_day, ntp_hour, ntp_min, ntp_sec));
    Serial.println("RTC Guncellendi.");
  }
  else {
    Serial.println("SIM kart modulunden guncel zaman alinamadi.");
  }
}

bool SIMStatusControl() { //Function to check if cellular data is available
  return modem.isGprsConnected() == 1;
}

bool IsHashEqualWiFi() { //Function to check if hash from memory and server are the same with Wi-Fi
  HTTPClient http;
  http.begin(hashAddress);
  http.addHeader("Authorization", "Bearer " + String(bearerToken));
  http.setTimeout(10000);
  int httpResponseCode = http.GET();
  if (httpResponseCode == 200) {
    String getHash;
    String payload = http.getString();
    if (payload.length() > 0) {
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);
      if (!error) {
        const char* hashValue = doc["Hash"];
        getHash = String(hashValue);
        newHash = getHash;
      }
      else {
        Serial.print("JSON parse hatasi! Hafizadaki kartlar ile devam edilecek. Hata: "); Serial.println(error.c_str());
        return true;
      }
    }
    String lastHash = GetLastHash();
    Serial.println("get hash: " + getHash + ", last hash: " + lastHash);
    http.end();
    if (getHash != lastHash) {
      Serial.println("Yeni kartlar cekilip hafizaya yazilacak.");
      return false;
    }
    else {
      Serial.println("Hafizadaki kartlarla devam edilecek.");
      return true;
    }
  }
  else {
    Serial.printf("ishashequal GET istegi basarisiz, hata: %s\n", http.errorToString(httpResponseCode).c_str());
    Serial.println("Hafizadaki kartlar ile devam edilecek.");
    http.end();
    return true;
  }
}

bool FetchAndSaveJsonWiFi() { //Function to fetch new JSON file from the server Wi-Fi
  const char* filePath = "";
  EEPROM.get(eepromAddress, valueRead);
  if (valueRead == 1) {
    filePath = "/large2.json";
  }
  else {
    filePath = "/large1.json";
  }

  Serial.print("JSON suraya cekilecek: "); Serial.println(filePath);
  HTTPClient http;
  http.begin(getAddress);
  http.addHeader("Authorization", "Bearer " + String(bearerToken));
  http.setTimeout(30000);
  int httpCode = http.GET();
  
  if (httpCode < 200 || httpCode > 299) {
    Serial.printf("FetchAndSaveJsonWiFi GET istegi basarisiz, hata: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return false;
  }

  int contentLength = http.getSize();
  Serial.print("FetchAndSaveJsonWiFi icerik uzunlugu: "); Serial.println(contentLength);

  if (contentLength <= 0) {
    Serial.println("FetchAndSaveJsonWiFi gecersiz icerik uzunlugu.");
    http.end();
    return false;
  }

  File file = SPIFFS.open(filePath, FILE_WRITE);
  if (!file) {
    Serial.println("FetchAndSaveJsonWiFi SPIFFS dosyasi acilamadi.");
    http.end();
    return false;
  }

  WiFiClient* stream = http.getStreamPtr();
  uint8_t buff[256] = { 0 };
  int totalBytes = 0;

  while (http.connected() && (stream->available() || totalBytes < contentLength)) {
    int len = stream->readBytes(buff, sizeof(buff));
    if (len > 0) {
      file.write(buff, len);
      totalBytes += len;
    }
    else {
      Serial.println("FetchAndSaveJsonWiFi baglanti sirasinda veri okunamadi.");
      file.close();
      http.end();
      return false;
    }
  }

  file.close();
  http.end();

  Serial.print("Cekilen toplam byte: "); Serial.println(totalBytes);

  if (totalBytes == 0 || totalBytes != contentLength) {
    Serial.println("Guncel JSON tamamen cekilemedi. Islem basarisiz.");
    return false;
  }

  if (ChangeHash(newHash)) {
    Serial.println("Hafizadaki hash degistirildi.");
  }
  else {
    Serial.println("Hafizadaki hash degistirilemedi.");
    return false;
  }
  
  if (valueRead == 1) {
    EEPROM.put(eepromAddress, 2);
  }
  else {
    EEPROM.put(eepromAddress, 1);
  }

  if (EEPROM.commit()) {
    EEPROM.get(eepromAddress, valueRead);
    Serial.print(valueRead); Serial.println(" degeri basariyla EEPROM'a yazildi.");
  }
  else {
    Serial.println("EEPROM'a veri yazma basarisiz.");
    return false;
  }
  update = true;
  return true;
}

bool IsHashEqualSIM() { //Function to check if hash from memory and server are the same with cellular data
  HttpClient httpCardHash(secure_layer, mainServer, 443);
  httpCardHash.beginRequest();
  httpCardHash.get(hashAddressPath);
  httpCardHash.sendHeader("Authorization", String("Bearer ") + bearerToken);
  httpCardHash.endRequest();
  int statusCode = httpCardHash.responseStatusCode();
  int contentLength = httpCardHash.contentLength();
  if (statusCode == 200) {
    String getHash;
    String payload = httpCardHash.responseBody();
    if (payload.length() > 0) {
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);
      if (!error) {
        const char* hashValue = doc["Hash"];
        getHash = String(hashValue);
        newHash = getHash;
      }
      else {
        Serial.print("JSON parse hatasi! Hafizadaki kartlar ile devam edilecek. Hata: "); Serial.println(error.c_str());
        return true;
      }
    }
    String lastHash = GetLastHash();
    Serial.println("get hash: " + getHash + ", last hash: " + lastHash);
    httpCardHash.stop();
    if (getHash != lastHash) {
      Serial.println("Yeni kartlar cekilip hafizaya yazilacak.");
      return false;
    }
    else {
      Serial.println("Hafizadaki kartlarla devam edilecek.");
      return true;
    }
  }
  else {
    Serial.printf("ishashequal GET istegi basarisiz, hata: %d\n", statusCode);
    Serial.println("Hafizadaki kartlar ile devam edilecek.");
    httpCardHash.stop();
    return true;
  }
}

bool FetchAndSaveJsonSIM() { //Function to fetch new JSON file from the server cellular data
  const char* filePath = "";
  EEPROM.get(eepromAddress, valueRead);
  if (valueRead == 1) {
    filePath = "/large2.json";
  }
  else {
    filePath = "/large1.json";
  }
  Serial.printf("JSON suraya cekilecek: %s\n", filePath);

  HttpClient httpGetCards(secure_layer, mainServer, 443);
  httpGetCards.beginRequest();
  httpGetCards.get(getAddressPath);
  httpGetCards.sendHeader("Authorization", String("Bearer ") + bearerToken);
  httpGetCards.sendHeader("Connection", "keep-alive");
  httpGetCards.sendHeader("Keep-Alive", "timeout=60, max=1000");
  httpGetCards.endRequest();
  int httpCode = httpGetCards.responseStatusCode();
  int contentLength = httpGetCards.contentLength();
  
  if (httpCode != 200) {
    Serial.printf("FetchAndSaveJsonSIM GET istegi basarisiz, hata: %d\n", httpCode);
    httpGetCards.stop();
    return false;
  }

  Serial.print("FetchAndSaveJsonSIM icerik uzunlugu: "); Serial.println(contentLength);

  if (contentLength <= 0) {
    Serial.printf("FetchAndSaveJsonSIM gecersiz icerik uzunlugu: %d.\n", contentLength);
    httpGetCards.stop();
    return false;
  }

  File file = SPIFFS.open(filePath, FILE_WRITE);
  if (!file) {
    Serial.println("FetchAndSaveJsonSIM SPIFFS dosyasi acilamadi.");
    httpGetCards.stop();
    return false;
  }

  const int bufferSize = 1024;
  uint8_t buffer[bufferSize];

  int bytesRead = 0;
  while (httpGetCards.connected() && bytesRead < contentLength) {
    int readSize = httpGetCards.read(buffer, sizeof(buffer));
    //int readSize = httpGetCards.readBytes(buffer, sizeof(buffer));
    if (readSize > 0) {
      file.write(buffer, readSize);
      Serial.println(bytesRead);
      bytesRead += readSize;
    }
    else {
      break; //Verinin tamamı alındı
      Serial.println("break kosulu");
    }
    delay(10);
  }

  file.close();
  httpGetCards.stop();

  Serial.print("cekilen toplam byte: "); Serial.println(bytesRead);

  if (bytesRead == 0 || bytesRead != contentLength) {
    Serial.println("Guncel JSON tamamen cekilemedi. Islem basarisiz.");
    return false;
  }

  if (ChangeHash(newHash)) {
    Serial.println("Hafizadaki hash degistirildi.");
  }
  else {
    Serial.println("Hafizadaki hash degistirilemedi.");
    return false;
  }
  
  if (valueRead == 1) {
    EEPROM.put(eepromAddress, 2);
  }
  else {
    EEPROM.put(eepromAddress, 1);
  }

  if (EEPROM.commit()) {
    EEPROM.get(eepromAddress, valueRead);
    Serial.print(valueRead); Serial.println(" degeri basariyla EEPROM'a yazildi.");
  }
  else {
    Serial.println("EEPROM'a veri yazma basarisiz.");
    return false;
  }
  return true;
}

String GetLastHash() { //Function to get hash code in memory
  bool openedHashFile = false;
  File hashFile = SPIFFS.open(hashFileName, "r");
  if (hashFile) {
    openedHashFile = true;
  }
  String returnString = "";
  if (hashFile.available() && openedHashFile) {
    returnString = hashFile.readString();
  }
  if (openedHashFile) {
    hashFile.close();
    return returnString;
  }
  if (!openedHashFile) {
    returnString = "could not open file";
    return returnString;
  }
}

bool ChangeHash(String getHash){ //Function to replace the hash code in memory with the new hash code on the server.
  File hashFile = SPIFFS.open(hashFileName, FILE_WRITE); 
  if (!hashFile) {
    Serial.println("hash dosyasi yazma icin acilamadi.");
    return false;
  }
  if (hashFile.available()) {
    if (hashFile.print(getHash)) {
      hashFile.close();
      return true;
    }
    else {
      Serial.println("yeni hash dosyaya yazilamadi.");
      hashFile.close();
      return false;
    }
  }
}

String SerializeToGPSJson(float lat, float lon, float speed, unsigned long unixtime) { //Function to serialize GPS location data to JSON file format
  speed = speed * 1.852;
  String output = "{\"lat\":\"" + String(lat,8) + "\",\"lon\":\"" + String(lon,8) + "\",\"speed\":\"" + String(speed,8) + "\",\"imei\":\"" + imei + "\",\"unixtime\":\"" + String(unixtime) + "\"}";
  return output;
}

bool IsThereNotSentCards(size_t &readSize) { //Function to check if there are not sent cards in flash memory
  File notSentCardsFile = SPIFFS.open(notSentCardsFileName, FILE_READ);
  if (!notSentCardsFile) {
    Serial.println("notSentCardsFile dosyasi kontrol icin acilamadi.");
    return false;
  }
  if (notSentCardsFile.available()) {
    if(notSentCardsFile.size() == 0){
      notSentCardsFile.close();
      return false;
    }
    else if (notSentCardsFile.size() != 0) {
      readSize = notSentCardsFile.size();
      notSentCardsFile.close();
      return true;
    }
  }
  else {
    notSentCardsFile.close();
    return false;
  }
}

bool GetNotSentCardsFromMemory(String &notSentCardsString, size_t fileSize) { //Function to get not sent cards from memory to post to server
  File notSentCardsFile = SPIFFS.open(notSentCardsFileName, FILE_READ);
  if (!notSentCardsFile) {
    Serial.println("notSentCardsFile okuma icin acilamadi");
    return false;
  }
  while (notSentCardsFile.available()) {
    notSentCardsString += (char)notSentCardsFile.read();
  }
  if (notSentCardsString.length() == fileSize) { //Whole file is read
    return true;
  }
  else {
    Serial.println("Dosya eksik okundu.");
    return false;
  }
}

String SerializeToJson(const String& input) { //Function to serialize ReadCard struct to JSON file format
  if (input.length() != 19) {
    return "{\"CardId\":\"FFFFFFFF\",\"imei\":\"" + imei + "\",\"ReadDate\":\"0000000000\",\"Auth\":\"0\"}";  //In case of an invalid length string, invalid JSON is returned.
  }
  StaticJsonDocument<200> doc;
  doc["CardId"] = input.substring(0, 8);
  doc["imei"] = imei;
  doc["ReadDate"] = input.substring(8, 18);
  if (input.substring(18, 19) == "1") {
    doc["Auth"] = "Valid";
  }
  else {
    doc["Auth"] = "Declined";
  }
  String output;
  serializeJson(doc, output);
  return output;
}

bool SaveNotSentCardsToMemory(String notSentCardsString) { //Function to save not sent cards to memory to send later
  File notSentCardsFile = SPIFFS.open(notSentCardsFileName, FILE_WRITE);
  if (!notSentCardsFile) {
    Serial.println("notSentCardsFile yazma icin acilamadi");
    return false;
  }
  Serial.println("notSentCardsString: " + notSentCardsString);
  if (notSentCardsString.length() != 0) {
    if (!notSentCardsFile.print(notSentCardsString)) {
      Serial.println("notSentCardsString, notSentCardsFile'a yazilamadi.");
      notSentCardsFile.close();
      return false;
    }
    else {
      Serial.println("notSentCardsString, basariyla notSentCardsFile'a yazildi.");
      notSentCardsFile.close();
      return true;
    }
  }
  else {
    notSentCardsFile.seek(0);
    notSentCardsFile.print("");
    Serial.println("notSentCardsString bos oldugu icin dosyanin icerisi bosaltildi.");
    notSentCardsFile.close();
    return true;
  }
}

// <<<<<<<<<<<<<<<<<<<< LCD Functions >>>>>>>>>>>>>>>>>>>> //

void LCDInfo() { //Function to print "please wait" screen to LCD Screen while module is starting up
  PrintLogos(simstatus, WiFiAv, GPSAv);
  lcd.setCursor(1,1);
  lcd.print("Cankaya University");
  lcd.setCursor(2,2);
  lcd.print("Lutfen  Bekleyin");
  lcd.setCursor(4,3);
  lcd.print("Baslatiliyor");
}

void PrintLogos(int simstatus, bool WiFiAv, bool GPSAv) { //Function to print logos and date to first row of LCD Screen
  DateTime now = rtc.now();
  char buffer[20];
  sprintf(buffer, "%02d.%02d.%04d", now.day(), now.month(), now.year());
  lcd.setCursor(0, 0);
  lcd.print(buffer);
  if (simstatus == 0 && WiFiAv){
    lcd.setCursor(17,0);
    lcd.print(" ");
    lcd.setCursor(18,0);
    lcd.write(byte(5));
    lcd.setCursor(19,0);
    lcd.write(byte(0));
  }
  else if (simstatus == 1 && WiFiAv && GPSAv){
    lcd.setCursor(17,0);
    lcd.write(byte(1));
    lcd.setCursor(18,0);
    lcd.write(byte(2));
    lcd.setCursor(19,0);
    lcd.write(byte(0));
  }
  else if (simstatus == 1 && WiFiAv && !GPSAv){
    lcd.setCursor(17,0);
    lcd.print(" ");
    lcd.setCursor(18,0);
    lcd.write(byte(2));
    lcd.setCursor(19,0);
    lcd.write(byte(0));
  }
  else if (simstatus == 2 && WiFiAv && GPSAv){
    lcd.setCursor(17,0);
    lcd.write(byte(1));
    lcd.setCursor(18,0);
    lcd.write(byte(3));
    lcd.setCursor(19,0);
    lcd.write(byte(0));
  }
  else if (simstatus == 2 && WiFiAv && !GPSAv){
    lcd.setCursor(17,0);
    lcd.print(" ");
    lcd.setCursor(18,0);
    lcd.write(byte(3));
    lcd.setCursor(19,0);
    lcd.write(byte(0));
  }
  else if (simstatus == 3 && WiFiAv && GPSAv){
    lcd.setCursor(17,0);
    lcd.write(byte(1));
    lcd.setCursor(18,0);
    lcd.write(byte(4));
    lcd.setCursor(19,0);
    lcd.write(byte(0));
  }
  else if (simstatus == 3 && WiFiAv && !GPSAv){
    lcd.setCursor(17,0);
    lcd.print(" ");
    lcd.setCursor(18,0);
    lcd.write(byte(4));
    lcd.setCursor(19,0);
    lcd.write(byte(0));
  }
  else if (simstatus == 0 && !WiFiAv){
    lcd.setCursor(17,0);
    lcd.print(" ");
    lcd.setCursor(18,0);
    lcd.write(byte(5));
    lcd.setCursor(19,0);
    lcd.write(byte(6));
  }
  else if (simstatus == 1 && !WiFiAv && GPSAv){
    lcd.setCursor(17,0);
    lcd.write(byte(1));
    lcd.setCursor(18,0);
    lcd.write(byte(2));
    lcd.setCursor(19,0);
    lcd.write(byte(6));
  }
  else if (simstatus == 1 && !WiFiAv && !GPSAv){
    lcd.setCursor(17,0);
    lcd.print(" ");
    lcd.setCursor(18,0);
    lcd.write(byte(2));
    lcd.setCursor(19,0);
    lcd.write(byte(6));
  }
  else if (simstatus == 2 && !WiFiAv && GPSAv){
    lcd.setCursor(17,0);
    lcd.write(byte(1));
    lcd.setCursor(18,0);
    lcd.write(byte(3));
    lcd.setCursor(19,0);
    lcd.write(byte(6));
  }
  else if (simstatus == 2 && !WiFiAv && !GPSAv){
    lcd.setCursor(17,0);
    lcd.print(" ");
    lcd.setCursor(18,0);
    lcd.write(byte(3));
    lcd.setCursor(19,0);
    lcd.write(byte(6));
  }
  else if (simstatus == 3 && !WiFiAv && GPSAv){
    lcd.setCursor(17,0);
    lcd.write(byte(1));
    lcd.setCursor(18,0);
    lcd.write(byte(4));
    lcd.setCursor(19,0);
    lcd.write(byte(6));
  }
  else if (simstatus == 3 && !WiFiAv && !GPSAv){
    lcd.setCursor(17,0);
    lcd.print(" ");
    lcd.setCursor(18,0);
    lcd.write(byte(4));
    lcd.setCursor(19,0);
    lcd.write(byte(6));
  }
}

void LcdMenu() { //Function to print main menu to LCD Screen
  lcd.clear();
  lcd.noBlink();
  PrintLogos(simstatus, WiFiAv, GPSAv);
  lcd.setCursor(1,1);
  lcd.print("Cankaya University");
  lcd.setCursor(2,3);
  lcd.print("Kartinizi Okutun");
  strip.clear();
}

void LcdWait() { //Function to print a different waiting menu to LCD Screen
  lcd.clear();
  PrintLogos(simstatus, WiFiAv, GPSAv);
  lcd.setCursor(1,1);
  lcd.print("Cankaya University");
  lcd.setCursor(2,3);
  lcd.print("Lutfen  Bekleyin");
  strip.clear();
}

void LcdValidScreen(String name) { //Function to print card holder's name to LCD Screen
  lcd.clear();
  PrintLogos(simstatus, WiFiAv, GPSAv);
  lcd.setCursor(0,1);
  lcd.print("Sayin");
  lcd.setCursor(0,2);
  lcd.print(name);
  lcd.setCursor(4,3);
  lcd.print("Hos Geldiniz");
  for (int i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(0, 50, 0)); //green light
    strip.show();
  }
  digitalWrite(BUZZER_PIN, HIGH);
  delay(50);
  digitalWrite(BUZZER_PIN, LOW);
  delay(20);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(50);
  digitalWrite(BUZZER_PIN, LOW);
  strip.clear();
  strip.show();
}

void LcdDeclinedScreen() { //Function to print "invalid card" to LCD Screen
  lcd.clear();
  PrintLogos(simstatus, WiFiAv, GPSAv);
  lcd.setCursor(6, 2);
  lcd.print("GECERSIZ");
  lcd.setCursor(8, 3);
  lcd.print("KART");
  for (int i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, strip.Color(50, 0, 0)); //red light
    strip.show();
  }
  digitalWrite(BUZZER_PIN, HIGH);
  delay(600);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(600);
  digitalWrite(BUZZER_PIN, LOW);
  delay(100);
  digitalWrite(BUZZER_PIN, HIGH);
  delay(600);
  digitalWrite(BUZZER_PIN, LOW);
  strip.clear();
  strip.show();
}
