#include "BluetoothSerial.h"
#include <HardwareSerial.h>
#include "HardwareSerial_NB_BC95.h"
#include <SDI12.h>
#include "FS.h"
#include "SD.h"
#include <TaskScheduler.h>
#include <TimeLib.h>
#include <ArduinoJson.h>
#include "SPI.h"
#include <TFT_eSPI.h>
#include "Sensor.h"
#include "Free_Fonts.h"
#include <EEPROM.h>
#include <ModbusMaster.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <Wire.h>
#include <WiFiManager.h>


#include "rainImg.h"
#include "levelImg.h"
#include "batteryImg.h"
#include "wifi4.h"
#include "wifi3.h"
#include "wifi2.h"
#include "wifi1.h"

#define WIFI_AP ""
#define WIFI_PASSWORD ""
WiFiManager wifiManager;

#define _TASK_TIMECRITICAL

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define SCK  19
#define MISO  12
#define MOSI  13
#define CS  18

TFT_eSPI tft = TFT_eSPI();

#define SERIAL_BAUD 115200 /*!< The baud rate for the output serial port */
#define DATA_PIN 13         /*!< The pin of the SDI-12 data bus */
#define POWER_PIN -1       /*!< The sensor power pin (or -1 if not switching power) */
#define SENSOR_ADDRESS 0
#define DRY_CONTACT_PIN  25

String deviceToken = "wupnG4krB1CdhCIUk6W2";
String serverIP = "103.27.203.83"; // Your Server IP;
String serverPort = "19956"; // Your Server Port;
String json = "";
String udpData = "";
boolean stateGetWaterLevel = 0;
int prevStage = 0;
int rainGate = 0;

ModbusMaster node;
String waterLevel = "";
String voltLevel = "";

int reading;                // ค่าที่อ่านได้จากปุ่มกด (รวม bounce)
int counter = 0;            // จำนวน iteration ที่เราเห็นการกด หรือปล่อย
int current_state = LOW;    // ค่าที่ได้หลังการทำ debounce (คือการกดหรือปล่อยจริงๆ)
long timeLoop = 0;              // เวลาล่าสุดที่มีการเก็บค่า
int debounce_count = 2;    // จำนวนมิลลิวินาที/รอบการวนลูป ที่ต่ำสุดที่เชื่อได้ว่ามีการกด หรือปล่อยจริงๆ
int rainCount = 0;       // ไว้แสดงจำนวนการกด

int period = 2;
unsigned long time_now = 0;

HardwareSerial modbus(2);
HardwareSerial_NB_BC95 AISnb;
signal meta ;

BluetoothSerial SerialBT;
/** Define the SDI-12 bus */
SDI12 mySDI12(DATA_PIN);

String sdiResponse = "";
String myCommand   = "";
String model = "";
String result = "";
String _minusSign = "-";
unsigned long epoch;
unsigned long epoch_mill;
bool sendFinish = false;

StaticJsonDocument<400> doc;

const long interval = 300000;  //millisecond
unsigned long previousMillis = 0;

unsigned long lastSend;

const long intervalDrycontact = 1000;  //millisecond
unsigned long previousMillisDrycontact = 0;
void t1CallgetWaterLevel();
void t2CallgetVoltLevel();
void t3CallgetRain();

void t4Restart();
//TASK
//Task t1(60000, TASK_FOREVER, &t1CallgetMeter);
//Task t2(360000, TASK_FOREVER, &t2CallsendViaNBIOT);

 

Scheduler runner;
String _config = "{\"_type\":\"retrattr\",\"Tn\":\"8966031840041733110\",\"keys\":[\"epoch\",\"ip\"]}";
unsigned long _epoch = 0;
String _IP = "";
String dataJson = "";
boolean validEpoc = false;


//TASK
Task t1(60000, TASK_FOREVER, &t1CallgetWaterLevel);
Task t2(60000, TASK_FOREVER, &t2CallgetVoltLevel);
//Task t3(50, TASK_FOREVER, &t3CallgetRain);


struct Meter
{
  String waterLevel;
  String flow;
  String velocity;
  String area;
  String volt;

};
Meter meter;

unsigned long ms;




/**********************************************  WIFI Client 注意编译时要设置此值 *********************************
   wifi client
*/
//WiFi&OTA 参数
String HOSTNAME = "Decode-";
#define PASSWORD "7650" //the password for OTA upgrade, can set it in any char you want

void configModeCallback (WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  //if you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

void setup() {
  HeartBeat();
  Serial.begin(SERIAL_BAUD);
  modbus.begin(9600, SERIAL_8N1, 16, 17);
  initTFT();
  SerialBT.begin("Decode"); //Bluetooth device name
  SerialBT.println("Hello:Decode");
  while (!Serial)
    ;

  Serial.println("Opening SDI-12 bus...");
  mySDI12.begin();
  delay(1000);  // allow things to settle
  // Power the sensors;
  if (POWER_PIN > 0) {
    Serial.println("Powering up sensors...");
    pinMode(POWER_PIN, OUTPUT);
    digitalWrite(POWER_PIN, HIGH);
    delay(200);
  }
  Serial.print("devie:");
  Serial.println(SENSOR_ADDRESS);
  SerialBT.print("devie:");
  SerialBT.println(SENSOR_ADDRESS);
  getModel();

  AISnb.debug = true;
  AISnb.setupDevice(serverPort);

  String ip1 = AISnb.getDeviceIP();
  delay(1000);

  pingRESP pingR = AISnb.pingIP(serverIP);

  Serial.println(model);
  pinMode(DRY_CONTACT_PIN, INPUT_PULLUP);
  attachInterrupt(DRY_CONTACT_PIN, checkRainGate, CHANGE);
  String nccid = AISnb.getNCCID();
  //  runner.init();
  Serial.println("Initialized scheduler");

  runner.addTask(t1);
  Serial.println("added t1");
  runner.addTask(t2);
  Serial.println("added t2");
  //  runner.addTask(t3);
  //  Serial.println("added t3");
  delay(2000);
  t1.enable();  Serial.println("Enabled t1");
  t2.enable();  Serial.println("Enabled t2");
  //  t3.enable();  Serial.println("Enabled t3");

  Serial.print("nccid:");
  Serial.println(nccid);

  SerialBT.println("Start..");
  initSD();
  tft.fillScreen(TFT_WHITE);
  tft.drawString("Wait for Wifi Setting (Timeout 60 Sec)", tft.width() / 2, tft.height() / 2, GFXFF);
  wifiManager.setTimeout(60);
  wifiManager.setAPCallback(configModeCallback);
  String wifiName = "@Decode-";
  wifiName.concat(String((uint32_t)ESP.getEfuseMac(), HEX));
  if (!wifiManager.autoConnect(wifiName.c_str())) {
    //Serial.println("failed to connect and hit timeout");
    //reset and try again, or maybe put it to deep sleep
    //    ESP.reset();
    //delay(1000);
  }

  HeartBeat();
  setupOTA();
  getepoch();
  tft.fillScreen(TFT_BLACK);
  lastSend = epoch + ((millis() - epoch_mill) / 1000) + (7 * 3600);
  TFTshow(epoch + ((millis() - epoch_mill) / 1000) + (7 * 3600));



  runner.init();
  Serial.println("Initialized scheduler");

//  runner.addTask(t1);
//  Serial.println("added t1");
//  runner.addTask(t2);
//  Serial.println("added t2");
  //  runner.addTask(t3);
  //  Serial.println("added t3");
  HeartBeat();
  delay(2000);
//  t1.enable();  Serial.println("Enabled t1");
//  t2.enable();  Serial.println("Enabled t2");
  //  t3.enable();  Serial.println("Enabled t3");
  

  HeartBeat();
  HOSTNAME.concat(getMacAddress());
  SerialBT.begin(HOSTNAME); //Bluetooth

  SerialBT.begin(HOSTNAME); //Bluetooth device name
  SerialBT.println(HOSTNAME);
  AISnb.debug = true;
  AISnb.setupDevice(serverPort);
  HeartBeat();
  _init();
  HeartBeat();
  _loadConfig();

  Serial.println();
  Serial.println(F("***********************************"));

  Serial.println("Initialize...");
  HeartBeat();
}

void loop() {
  ArduinoOTA.handle();
  runner.execute();
  /*unsigned long nowtime = epoch + ((millis() - epoch_mill) / 1000) + (7 * 3600);
  //unsigned long nowtime = epoch + millis() - epoch_mill;
  Serial.print("Minute : ");
  Serial.println(minute(nowtime));
  TFTshow(nowtime);
  //  delay(20000);
  if (minute(nowtime) % 5 == 0 && sendFinish == false) {
    appendSD(nowtime);
    meta = AISnb.getSignal();
    udpData = "{\"Tn\":\"";
    udpData.concat(deviceToken);
    udpData.concat("\",\"level\":");
    waterLevel = "2";
    udpData.concat(waterLevel);
    udpData.concat(",\"rssi\":");
    udpData.concat(meta.rssi);
    udpData.concat(",\"rain\":");
    udpData.concat(rainCount);
    udpData.concat(",\"volt\":");
    udpData.concat(voltLevel);
    udpData.concat("}");
    Serial.println(udpData);
    SerialBT.print("send:");
    SerialBT.println(udpData);
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, udpData);
    Serial.print("udp.status:");
    Serial.println(udp.status);
    //delay(10000);
    if (udp.status) {
      lastSend = nowtime;
      Serial.println("reset counter rain");
      rainCount = 0;
    }
    sendFinish = true;
  } else if (minute(nowtime) % 15 != 0) {
    sendFinish = false;
  }
  delay(1000);*/
}

void t1CallgetWaterLevel() {
  getWaterLevel();
}

void t2CallgetVoltLevel() {
  checkVoltLevel();
}

void t3CallgetRain() {
  checkRainGate();
}

void getModel() {
  Serial.println("start query..");
  myCommand = String(SENSOR_ADDRESS) + "I!";

  Serial.print("cmd:");
  Serial.println(myCommand);  // echo command to terminal

  mySDI12.sendCommand(myCommand);
  delay(1000);  // wait a while for a response

  while (mySDI12.available()) {  // build response string
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }

  if (sdiResponse.length() > 1) {
    Serial.println(sdiResponse);  // write the response to the screen
    model = sdiResponse;
  }
  mySDI12.clearBuffer();

  delay(200);       // delay between taking reading and requesting data

  // next command to request data from last measurement
  myCommand = String(SENSOR_ADDRESS) + "D0!";
  Serial.println(myCommand);  // echo command to terminal

  mySDI12.sendCommand(myCommand);
  delay(300);  // wait a while for a response

  while (mySDI12.available()) {  // build string from response
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }
  if (sdiResponse.length() > 1) {
    Serial.println(sdiResponse);  // write the response to the screen
    model = sdiResponse;
  }
  mySDI12.clearBuffer();
}

float voltMeasure(int Pin)
{
  unsigned int vRAW = 0;
  float Vout = 0.0;
  float Vin = 0.0;
  float R1 = 15000.0;
  float R2 = 2000.0;

  vRAW = analogRead(Pin);
  Vout = (vRAW * 3.3) / 4096;
  Vin = Vout / (R2 / (R1 + R2));
  if (Vin < 0.05)
  {
    Vin = 0.0;
  }
  return Vin + 1;
}

boolean getResponse() {
}

void getWaterLevel() {
  sdiResponse = "";  // clear the response string

  Serial.println("start query..");
  myCommand = String(SENSOR_ADDRESS) + "M!";
  Serial.print("cmd:");
  Serial.println(myCommand);  // echo command to terminal

  mySDI12.sendCommand(myCommand);
  delay(12000);  // wait a while for a response

  while (mySDI12.available()) {  // build response string
    char c = mySDI12.read();
    if ((c != '\n') && (c != '\r')) {
      sdiResponse += c;
      delay(5);
    }
  }
  Serial.print("  sdiResponse:");
  Serial.println(sdiResponse);  // write the response to the screen

  if (! sdiResponse.indexOf("121") > 0 ) {

    int whereis_ = sdiResponse.indexOf("+");
    Serial.println(whereis_);

    if (whereis_ > 0) {
      Serial.print("  +:");
      waterLevel = sdiResponse.substring(whereis_, sdiResponse.length());
      waterLevel = string2float(waterLevel);
      Serial.println(waterLevel);
    } else {
      whereis_ = sdiResponse.indexOf("-");
      if (whereis_ > 0 ) { // check for - value
        Serial.print("  -:");
        waterLevel = sdiResponse.substring(whereis_, sdiResponse.length());
        waterLevel = string2float(waterLevel);
        Serial.println(waterLevel);
      }
    }
    mySDI12.clearBuffer();
  } else {
    delay(200);       // delay between taking reading and requesting data
    sdiResponse = "";  // clear the response string


    // next command to request data from last measurement
    myCommand = String(SENSOR_ADDRESS) + "D0!";
    Serial.print("cmd:");
    Serial.println(myCommand);  // echo command to terminal

    mySDI12.sendCommand(myCommand);
    delay(1200);  // wait a while for a response
    //    for (int i = 0; i < 1200000; i++);
    while (mySDI12.available()) {  // build string from response
      char c = mySDI12.read();
      if ((c != '\n') && (c != '\r')) {
        sdiResponse += c;
        delay(5);
      }
    }
    Serial.print("  sdiResponse:");
    Serial.println(sdiResponse);  // write the response to the screen
    if (sdiResponse.length() > 1) {
      int whereis_ = sdiResponse.indexOf("+");
      Serial.println(whereis_);
      if (whereis_ > 0) {
        Serial.print("  +:");
        waterLevel = sdiResponse.substring(whereis_, sdiResponse.length());
        waterLevel = string2float(waterLevel);
        Serial.println(waterLevel);
      } else {
        whereis_ = sdiResponse.indexOf("-");
        if (whereis_ > 0 ) { // check for - value
          Serial.print("  -:");
          waterLevel = sdiResponse.substring(whereis_, sdiResponse.length());
          waterLevel =  string2float(waterLevel);
          Serial.println(waterLevel);
        }
      }
    }
    mySDI12.clearBuffer();
  }
}

void checkVoltLevel()
{
  float voltlev;
  voltlev = voltMeasure(35);
  Serial.print("Battery");
  Serial.print(":");
  Serial.println(voltlev);
  voltLevel = String(voltlev, 2);
}

void checkRainGate()
{
  int current1_state, current2_state;
  //อ่านค่าปุ่มกด
  current1_state = digitalRead(DRY_CONTACT_PIN);

  time_now = millis();
  while (millis() < time_now + period) {
    //wait approx. [period] ms
  }
  current2_state = digitalRead(DRY_CONTACT_PIN);

  if (current1_state == current2_state)
  {
    //ถ้าเป็นการกดจะแสดงผลจำนวนที่กดไปทาง serial monitor
    if (current1_state == LOW) {
      rainCount++;
      //      Serial.print("LOW");
      Serial.print(rainCount);
      Serial.print(":");
      Serial.println(current_state);
      SerialBT.print(rainCount);
      SerialBT.print(":");
      SerialBT.println(current_state);
    }
    counter = 0;
    current_state = digitalRead(DRY_CONTACT_PIN);
    //    Serial.print("HIGH ");
    Serial.print(rainCount);
    Serial.print(":");
    Serial.println(current_state);
    SerialBT.print(rainCount);
    SerialBT.print(":");
    SerialBT.println(current_state);
  }
}

String string2float(String v) {
  String result = "";
  //+0145.715
  //+0105.715
  Serial.print("    :");
  Serial.println(v);

  if (v.indexOf("-") == 0)
    result = _minusSign;

  if (!(  v.startsWith("+0000", 0) == 0  && v.startsWith("-0000", 0)  == 0) ) {
    Serial.println("  5");
    result += "0" + v.substring(5, v.length());
    return result;
  } else if (! ( v.startsWith("+000", 0) == 0  && v.startsWith("-000", 0) == 0 ) ) {
    Serial.println("  4");
    result +=   v.substring(4, v.length());
    return result;
  } else if  (!( v.startsWith("+00", 0) == 0  && v.startsWith("-00", 0) == 0 ) ) {
    Serial.println("  3");
    result += v.substring(3, v.length());
    Serial.print("result:");
    Serial.println(result);
    return result;
  } else if  (! ( v.startsWith("+0", 0) == 0  && v.startsWith("-0", 0) == 0 ) ) {
    Serial.println("  2");
    result += v.substring(2, v.length());
    return result;
  } else if  (! ( v.startsWith("+", 0) == 0 && v.startsWith("-", 0) == 0 ) ) {
    Serial.println("  1");
    result += v.substring(1, v.length());
    return result;
  } else {
    Serial.println("  0");
    result += v.substring(0, v.length());
    return result;
  }
}

char  char_to_byte(char c)
{
  if ((c >= '0') && (c <= '9'))
  {
    return (c - 0x30);
  }
  if ((c >= 'A') && (c <= 'F'))
  {
    return (c - 55);
  }
}

void getepoch() {
  String retJson = "{\"_type\":\"retrattr\",\"Tn\":\"";
  retJson.concat(deviceToken);
  retJson.concat("\",\"keys\":[\"epoch\",\"ip\"]}");
  do {
    json = "";
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, retJson);
    UDPReceive resp = AISnb.waitResponse();
    epoch_mill = millis();
    Serial.println("Start Send");
    AISnb.receive_UDP(resp);
    Serial.print("waitData:");
    Serial.println(resp.data);
    if (udp.status == true) {
      for (int x = 0; x < resp.data.length(); x += 2) {
        char c = char_to_byte(resp.data[x]) << 4 | char_to_byte(resp.data[x + 1]);

        json += c;
      }
      //Serial.println(json);
      DeserializationError error = deserializeJson(doc, json);

      // Test if parsing succeeds.
      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        delay(4000);
      } else {
        epoch = doc["epoch"];
        Serial.print("epoch : ");
        Serial.println(epoch);
        break;
      }
    }
  } while (true);
}

void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void deleteFile(fs::FS &fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }
  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}

//void t2CallShowEnv() {
//
//}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print(" DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print(" FILE: ");
      Serial.print(file.name());
      Serial.print(" SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void initSD() {
  SPIClass spi = SPIClass(VSPI);
  spi.begin(SCK, MISO, MOSI, CS);
  if (!SD.begin(CS, spi, 80000000)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  //  writeFile(SD, "/hello.txt", "Hello ");
  //  appendFile(SD, "/hello.txt", "World!\n");
  //  readFile(SD, "/hello.txt");
  //  testFileIO(SD, "/test.txt");
  //  Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  //  Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  listDir(SD, "/", 0);
  if (!SD.exists("/data.csv")) {
    writeFile(SD, "/data.csv", "DateTime,waterLevel,rainCount,voltLevel,rssi\n");
  }
}

void initTFT() {
  tft.init();
  tft.setSwapBytes(true);
  tft.setRotation(1);
  tft.setFreeFont(FMB24);
  tft.fillScreen(TFT_WHITE);
  tft.setTextColor(TFT_BLACK);
  tft.drawString("Decode", tft.width() / 2, tft.height() / 2, GFXFF);
}

String a0(int n) {
  return (n < 10) ? "0" + String(n) : String(n);
}
String mkTime(unsigned long epoch_) {
  return String(year(epoch_)) + "-" + a0(month(epoch_)) + "-" + a0(day(epoch_)) + " " + a0(hour(epoch_)) + ":" + a0(minute(epoch_)) + ":" + a0(second(epoch_));
}

void TFTshow(unsigned long NowTime) {
  tft.setTextSize(1);
  tft.setFreeFont(FMB9);
  tft.setTextColor(TFT_WHITE);
  tft.fillRect(0,30,130,10,TFT_BLACK);
  tft.fillRect(170,10,100,30,TFT_BLACK);
  tft.drawString(mkTime(NowTime), 0,20);
  tft.setFreeFont(FMB9);
  tft.setTextSize(1);
  int rssi = map(meta.rssi.toInt(), -110, -40, 0, 100);
  if (rssi > 100) rssi = 100;
  if (rssi < 0) rssi = 0;
  if (rssi > 74) tft.pushImage(280, 10, WifiWidth, WifiHeight, wifi4);
  else if (rssi > 49) tft.pushImage(280, 10, WifiWidth, WifiHeight, wifi3);
  else if (rssi > 24) tft.pushImage(280, 10, WifiWidth, WifiHeight, wifi2);
  else tft.pushImage(280, 10, WifiWidth, WifiHeight, wifi1);
  
  tft.pushImage(210, 14, BatteryWidth, BatteryHeight, battery);
  tft.drawString(meter.volt+"V",240,20);
  tft.setFreeFont(FMB9);
  tft.drawString("ID:" + deviceToken,0,60);
  tft.pushImage(10, 90, RainWidth, RainHeight, rain);
  tft.fillRect(130,100,200,24,TFT_BLACK);
  tft.drawString("RAIN : "+ String(rainCount), 40,100);
  tft.pushImage(10, 134, LevelWidth, LevelHeight, level);
  tft.fillRect(140,140,190,24,TFT_BLACK);
  tft.drawString("LEVEL : "+waterLevel, 40,140);
  tft.setFreeFont(FMB9);
  tft.fillRect(0,215,320,25,TFT_RED);
  tft.drawString("SAVE :"+ mkTime(lastSend), 10,220);
}

void appendSD(unsigned long nowtime) {
  String Data = a0(day(nowtime)) + "/" + a0(month(nowtime)) + "/" + String(year(nowtime)) + " " + a0(hour(nowtime)) + ":" + a0(minute(nowtime)) + ":" + a0(second(nowtime));
  Data.concat(",");
  Data.concat(waterLevel);
  Data.concat(",");
  Data.concat(rainCount);
  Data.concat(",");
  Data.concat(voltLevel);
  Data.concat(",");
  Data.concat(meta.rssi);
  Data.concat("\n");
  char csvData[200];
  Data.toCharArray(csvData, Data.length() + 1);
  appendFile(SD, ("/data_" + a0(month(nowtime)) + "_" + String(year(nowtime)) + ".csv").c_str(), csvData);
}




void _writeEEPROM(String data) {
  Serial.print("Writing Data:");
  Serial.println(data);

  writeString(10, data);  //Address 10 and String type data
  delay(10);
}

void _loadConfig() {
  serverIP = read_String(10);
  serverIP.trim();
  Serial.print("IP:");
  Serial.println(serverIP);
}


//char  char_to_byte(char c)
//{
//  if ((c >= '0') && (c <= '9'))
//  {
//    return (c - 0x30);
//  }
//  if ((c >= 'A') && (c <= 'F'))
//  {
//    return (c - 55);
//  }
//}
void _init() {

  Serial.println(_config);

  do {
    UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, _config);
    dataJson = "";
    deviceToken = AISnb.getNCCID();
    Serial.print("nccid:");
    Serial.println(deviceToken);


    UDPReceive resp = AISnb.waitResponse();
    AISnb.receive_UDP(resp);
    Serial.print("waitData:");
    Serial.println(resp.data);


    for (int x = 0; x < resp.data.length(); x += 2)
    {
      char c =  char_to_byte(resp.data[x]) << 4 | char_to_byte(resp.data[x + 1]);

      dataJson += c;
    }
    Serial.println(dataJson);
    DeserializationError error = deserializeJson(doc, dataJson);

    // Test if parsing succeeds.
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      validEpoc = true;
      delay(4000);
    } else {
      validEpoc = false;
      unsigned long epoch = doc["epoch"];
      _epoch = epoch;
      String ip = doc["ip"];
      _IP = ip;
      Serial.println(dataJson);
      Serial.print("epoch:");  Serial.println(_epoch);
      _writeEEPROM(_IP);
      Serial.println(_IP);

    }
    delay(5000);
    HeartBeat();
  } while (validEpoc);


}


void writeString(char add, String data)
{
  EEPROM.begin(512);
  int _size = data.length();
  int i;
  for (i = 0; i < _size; i++)
  {
    EEPROM.write(add + i, data[i]);
  }
  EEPROM.write(add + _size, '\0'); //Add termination null character for String Data
  EEPROM.commit();
}


String read_String(char add)
{
  int i;
  char data[100]; //Max 100 Bytes
  int len = 0;
  unsigned char k;
  k = EEPROM.read(add);
  while (k != '\0' && len < 500) //Read until null character
  {
    k = EEPROM.read(add + len);
    data[len] = k;
    len++;
  }
  data[len] = '\0';
  Serial.print("Debug:");
  Serial.println(String(data));
  return String(data);
}



void setupOTA()
{
  //Port defaults to 8266
  //ArduinoOTA.setPort(8266);

  //Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(HOSTNAME.c_str());

  //No authentication by default
  ArduinoOTA.setPassword(PASSWORD);

  //Password can be set with it's md5 value as well
  //MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  //ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]()
  {
    Serial.println("Start Updating....");
    SerialBT.println("Start Updating....");
    HeartBeat();
    SerialBT.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");

    Serial.printf("Start Updating....Type:%s\n", (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem");
  });

  ArduinoOTA.onEnd([]()
  {

    SerialBT.println("Update Complete!");
    Serial.println("Update Complete!");


    ESP.restart();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    String pro = String(progress / (total / 100)) + "%";
    //    int progressbar = (progress / (total / 100));
    //int progressbar = (progress / 5) % 100;
    //int pro = progress / (total / 100);


    SerialBT.printf("Progress: %u%%\n", (progress / (total / 100)));

    Serial.printf("Progress: %u%%\n", (progress / (total / 100)));
    ms = millis();
    if (ms % 10000 == 0)
    {
      HeartBeat();

    }
  });

  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Error[%u]: ", error);
    String info = "Error Info:";
    switch (error)
    {
      case OTA_AUTH_ERROR:
        info += "Auth Failed";
        Serial.println("Auth Failed");
        break;

      case OTA_BEGIN_ERROR:
        info += "Begin Failed";
        Serial.println("Begin Failed");
        break;

      case OTA_CONNECT_ERROR:
        info += "Connect Failed";
        Serial.println("Connect Failed");
        break;

      case OTA_RECEIVE_ERROR:
        info += "Receive Failed";
        Serial.println("Receive Failed");
        break;

      case OTA_END_ERROR:
        info += "End Failed";
        Serial.println("End Failed");
        break;
    }


    Serial.println(info);
    ESP.restart();
  });

  ArduinoOTA.begin();
}

void setupWIFI()
{
  WiFi.setHostname(HOSTNAME.c_str());


  //等待5000ms，如果没有连接上，就继续往下
  //不然基本功能不可用
  byte count = 0;
  while (WiFi.status() != WL_CONNECTED && count < 10)
  {
    count ++;
    delay(500);
    Serial.print(".");
  }


  if (WiFi.status() == WL_CONNECTED)
    Serial.println("Connecting...OK.");
  else
    Serial.println("Connecting...Failed");

}

String getMacAddress() {
  uint8_t baseMac[6];
  // Get MAC address for WiFi station
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  char baseMacChr[18] = {0};
  sprintf(baseMacChr, "%02X%02X%02X%02X%02X%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  return String(baseMacChr);
}


//********************************************************************//
//*********************** HeartBeat Function **************************//
//********************************************************************//
void HeartBeat() {
  //   Sink current to drain charge from watchdog circuit
   
}

void t2CallsendViaNBIOT()
{
  meta = AISnb.getSignal();

  Serial.print("RSSI:"); Serial.println(meta.rssi);

  json = "";
  json.concat("{\"Tn\":\"");
  json.concat(deviceToken);
  /////
  json.concat(",\"waterLevel\":");
  json.concat(meter.waterLevel);
  json.concat("\",\"volt\":");
  json.concat(meter.volt);
  json.concat("\",\"flow\":");
  json.concat(meter.flow);
  json.concat("\",\"velocity\":");
  json.concat(meter.velocity);
  json.concat("\",\"area\":");
  json.concat(meter.area);
  json.concat(",\"rssi\":");
  json.concat(meta.rssi);
  json.concat(",\"csq\":");
  json.concat(meta.csq);
  json.concat("}");
  Serial.println(json);
  SerialBT.println(json);
  //
  UDPSend udp = AISnb.sendUDPmsgStr(serverIP, serverPort, json);
  UDPReceive resp = AISnb.waitResponse();
  Serial.print("rssi:");
  Serial.println(meta.rssi);
  SerialBT.print("rssi:");
  SerialBT.println(meta.rssi);
}

void readMeter()
{
  meter.waterLevel = read_Modbus(_waterLevel);
  delay(1000);
  meter.flow = read_Modbus(_flow);
  delay(1000);
  meter.velocity = read_Modbus(_velocity);
  delay(1000);
  meter.area = read_Modbus(_area);
  delay(1000);
  meter.volt = read_Modbus(_volt);
 
  Serial.print("meter.waterLevel:"); Serial.println( meter.waterLevel);
  Serial.print("meter.flow:"); Serial.println( meter.flow);  //Voltage Unbalance L-N Worst
  Serial.print("meter.velocity:"); Serial.println( meter.velocity);
  Serial.print("meter.area:"); Serial.println( meter.area);
  Serial.print("meter.volt:"); Serial.println( meter.volt);  //Voltage Unbalance L-N Worst

  Serial.println("");
  
}

void t1CallgetMeter() {     // Update read all data
  readMeter();
}

float HexTofloat(uint32_t x)
{

  return (*(float*)&x);
}


float read_Modbus(uint16_t  REG)
{
  static uint32_t i;
  uint32_t j, result;
  uint16_t data[2];
  uint32_t value = 0;
  float val = 0.0;

  // communicate with Modbus slave ID 1 over Serial (port 2)
  node.begin(4, modbus);

  // slave: read (6) 16-bit registers starting at register 2 to RX buffer
  result = node.readHoldingRegisters(REG, 2);
  Serial.print("result:");Serial.print(result); Serial.print(" node.ku8MBSuccess:");Serial.println(node.ku8MBSuccess);
  // do something with data if read is successful
  if (result == node.ku8MBSuccess)
  {
    for (j = 0; j < 2; j++)
    {
      data[j] = node.getResponseBuffer(j);
//      SerialBT.print(REG); SerialBT.print(":"); SerialBT.print(j); SerialBT.print(":");  SerialBT.println(data[j]);
      Serial.print(REG); Serial.print(":"); Serial.print(j); Serial.print(":");  Serial.println(data[j]);

    }
    value = data[0];
    value = value << 16;
    value = value + data[1];
 
    val = HexTofloat(value);


    return val;
  } else {
    Serial.print("Connec modbus fail. REG >>> "); Serial.println(REG, HEX); // Debug
    //    delay(1000);
    return 0;
  }
}
String decToHex(int decValue) {

  String hexString = String(decValue, HEX);
  return hexString;
}

unsigned int hexToDec(String hexString) {

  unsigned int decValue = 0;
  int nextInt;

  for (int i = 0; i < hexString.length(); i++) {

    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);

    decValue = (decValue * 16) + nextInt;
  }

  return decValue;
}
int getResult( unsigned int x_high, unsigned int x_low)
{
  String hex2 = "";
  hex2.concat(decToHex(x_low));
  hex2.concat(decToHex(x_high));
  Serial.print("hex:");  Serial.println(hex2);
  Serial.print("dec:");  Serial.println(hexToDec(hex2));                                                               //rightmost 8 bits
  return hexToDec(hex2);
}

/****************************************************
   [通用函数]ESP32 WiFi Kit 32事件处理
*/
void WiFiEvent(WiFiEvent_t event)
{
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event)
  {
    case SYSTEM_EVENT_WIFI_READY:               /**< ESP32 WiFi ready */
      break;
    case SYSTEM_EVENT_SCAN_DONE:                /**< ESP32 finish scanning AP */
      break;

    case SYSTEM_EVENT_STA_START:                /**< ESP32 station start */
      break;
    case SYSTEM_EVENT_STA_STOP:                 /**< ESP32 station stop */
      break;

    case SYSTEM_EVENT_STA_CONNECTED:            /**< ESP32 station connected to AP */
      break;

    case SYSTEM_EVENT_STA_DISCONNECTED:         /**< ESP32 station disconnected from AP */
      break;

    case SYSTEM_EVENT_STA_AUTHMODE_CHANGE:      /**< the auth mode of AP connected by ESP32 station changed */
      break;

    case SYSTEM_EVENT_STA_GOT_IP:               /**< ESP32 station got IP from connected AP */
    case SYSTEM_EVENT_STA_LOST_IP:              /**< ESP32 station lost IP and the IP is reset to 0 */
      break;

    case SYSTEM_EVENT_STA_WPS_ER_SUCCESS:       /**< ESP32 station wps succeeds in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_FAILED:        /**< ESP32 station wps fails in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_TIMEOUT:       /**< ESP32 station wps timeout in enrollee mode */
    case SYSTEM_EVENT_STA_WPS_ER_PIN:           /**< ESP32 station wps pin code in enrollee mode */
      break;

    case SYSTEM_EVENT_AP_START:                 /**< ESP32 soft-AP start */
    case SYSTEM_EVENT_AP_STOP:                  /**< ESP32 soft-AP stop */
    case SYSTEM_EVENT_AP_STACONNECTED:          /**< a station connected to ESP32 soft-AP */
    case SYSTEM_EVENT_AP_STADISCONNECTED:       /**< a station disconnected from ESP32 soft-AP */
    case SYSTEM_EVENT_AP_PROBEREQRECVED:        /**< Receive probe request packet in soft-AP interface */
    case SYSTEM_EVENT_AP_STA_GOT_IP6:           /**< ESP32 station or ap interface v6IP addr is preferred */
      break;

    case SYSTEM_EVENT_ETH_START:                /**< ESP32 ethernet start */
    case SYSTEM_EVENT_ETH_STOP:                 /**< ESP32 ethernet stop */
    case SYSTEM_EVENT_ETH_CONNECTED:            /**< ESP32 ethernet phy link up */
    case SYSTEM_EVENT_ETH_DISCONNECTED:         /**< ESP32 ethernet phy link down */
    case SYSTEM_EVENT_ETH_GOT_IP:               /**< ESP32 ethernet got IP from connected AP */
    case SYSTEM_EVENT_MAX:
      break;
  }
}
