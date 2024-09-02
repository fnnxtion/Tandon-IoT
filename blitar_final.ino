  #include <dummy.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Wire.h>
#include <DHT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_AHTX0.h>
#include <ModbusMasterPzem017.h>
static uint8_t pzemSlaveAddr = 0x01; //PZem Address
static uint16_t NewshuntAddr = 0x0002;      // Declare your external shunt value. Default is 100A, replace to "0x0001" if using 50A shunt, 0x0002 is for 200A, 0x0003 is for 300A
ModbusMaster node;
float PZEMVoltage = 0;
float PZEMCurrent = 0;
float PZEMPower = 0;
float PZEMEnergy = 0;

Adafruit_AHTX0 aht;

//Set Firebase Realtime Db
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#define WIFI_SSID "Spektrum Teknologi"
#define WIFI_PASSWORD "#2019gantisplas"
#define API_KEY "AIzaSyB8JR4DDaM-za0fPaZexwIhiamw1ukgKzM"
#define DATABASE_URL "https://projectpltsblitar-default-rtdb.asia-southeast1.firebasedatabase.app/"
#define USER_EMAIL "admin@esp32.com"
#define USER_PASSWORD "admin123"
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool taskCompleted = false;
FirebaseJson json;
String path = "Tandon-IoT";

const long utcOffsetInSeconds = 25200;
//----------------------------------------

//----------------------------------------Define NTP Client to get time
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
//----------------------------------------

unsigned long previousMillis = 0;        //--> will store last time was updated
const long interval = 1000;           //--> interval

//----------------------------------------Ultrasonic Sensor
#define trigPin 23    // HC-SR04 Trigger Pin at D24
#define echoPin 19    // HC-SR04 Echo Pin at D23
long durasi;          // Durasi
float jarak;          // Jarak

int wcond = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N2);
  setShunt(pzemSlaveAddr);
  node.begin(pzemSlaveAddr, Serial2);
  delay(1000);
  delay(500);
  pinMode (trigPin, OUTPUT);
  pinMode (echoPin, INPUT);
  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
    while (1) delay(10);
  }
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  config.api_key = API_KEY;
  auth.user.email = USER_EMAIL;
  auth.user.password = USER_PASSWORD;
  config.database_url = DATABASE_URL;
  config.token_status_callback = tokenStatusCallback; //see addons/TokenHelper.h
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
#if defined(ESP8266)
  fbdo.setBSSLBufferSize(1024, 1024);
#endif
  fbdo.setResponseSize(1024);
  Firebase.setReadTimeout(fbdo, 1000 * 60);
  Firebase.setwriteSizeLimit(fbdo, "tiny");
  Firebase.setFloatDigits(2);
  Firebase.setDoubleDigits(2);
}

void loop() {
  uint8_t result;
  result = node.readInputRegisters(0x0000, 6);
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis; //--> save the last time

    //----------------------------------------Update and send data every 5 seconds based on time.
    timeClient.update();
    int everyfive = timeClient.getSeconds();

    if (everyfive % 5 == 0) { //--> This means that NodeMCU sends and gets data at 0,5,10,15,20 seconds and so on.
      unsigned long epochTime = timeClient.getEpochTime();
      struct tm *ptm = gmtime ((time_t *)&epochTime);

      int monthDay = ptm->tm_mday;
      int currentMonth = ptm->tm_mon + 1;
      int currentYear = ptm->tm_year + 1900;
      String strmonthDay;
      String strcurrentMonth;

      if (monthDay < 10) {
        strmonthDay = "0" + String(monthDay);
      } else {
        strmonthDay = String(monthDay);
      }

      if (currentMonth < 10) {
        strcurrentMonth = "0" + String(currentMonth);
      } else {
        strcurrentMonth = String(currentMonth);
      }

      String currentDate = strmonthDay + "-" + strcurrentMonth + "-" + String(currentYear);
      Serial.print("Current Date: ");
      Serial.print(currentDate);

      String currentTime = timeClient.getFormattedTime();
      Serial.print("  Current Time: ");
      Serial.println(currentTime);

      //----------------------------------------Send Time Data to the Firebase Realtime Database.
      Firebase.setString(fbdo, "RealtimeDHTSensorAndLEDStatus/Time", currentTime); //-->
      Serial.println("Sending Time Data Successfully");
      Serial.println();

      // baca data sensor DHT11
      sensors_event_t humidity, temp;
      aht.getEvent(&humidity, &temp);// populate temp and humidity objects with fresh data

      int h = humidity.relative_humidity; //--> Read humidity.
      float t = temp.temperature; //--> Read temperature as Celsius (the default).
      // float tf = dht.readTemperature(true); //--> // Read temperature as Fahrenheit (isFahrenheit = true)

      // Check if any reads failed and exit early (to try again).
      if (isnan(h) || isnan(t)) {
        Serial.println(" Failed to load DHT sensor !");
        delay(1000);
        return;
      }
      //----------------------------------------

      Serial.print(F("Humidity: "));
      Serial.print(h);
      Serial.print(F("%  Temperature: "));
      Serial.print(t);
      Serial.println(F("°C "));

      String strHum = String(h); //--> Convert Humidity value to String data type.
      String strTem = String(t); //--> Convert Temperature values to the String data type.

      //---------------------------------------- Ultrasonic
      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(echoPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, HIGH);
      durasi = pulseIn(echoPin, HIGH);
      jarak = durasi * 0.034 / 2;
      double j = jarak;
      Serial.print("Jarak : ");
      Serial.print(j);
      float vol = 3.14 * 50 * 50 * 137;
      float cvol = 3.14 * 50 * 50 * jarak;
      float rvol = (vol - cvol) / 1000;
      if (jarak > 110 && jarak < 140) {
        Firebase.setString(fbdo, "Kondisi/air", 1);
        wcond = 1;
      } else if (jarak > 82 && jarak < 109) {
        Firebase.setString(fbdo, "Kondisi/air", 2);
        wcond = 2;
      } else if (jarak > 54 && jarak < 81) {
        Firebase.setString(fbdo, "Kondisi/air", 3);
        wcond = 3;
      } else if (jarak > 26 && jarak < 53) {
        Firebase.setString(fbdo, "Kondisi/air", 4);
        wcond = 4;
      } else if (jarak < 25) {
        Firebase.setString(fbdo, "Kondisi/air", 5);
        wcond = 5;
      }
      
      //---------------------------------------- PZEM-017 200A
      uint8_t result;
      result = node.readInputRegisters(0x0000, 6);
      if (result == node.ku8MBSuccess) {
        uint32_t tempdouble = 0x00000000;
        PZEMVoltage = node.getResponseBuffer(0x0000) / 100.0;
        PZEMCurrent = node.getResponseBuffer(0x0001) / 100.0;
        tempdouble =  (node.getResponseBuffer(0x0003) << 16) + node.getResponseBuffer(0x0002); // get the power value. Power value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit
        PZEMPower = tempdouble / 10.0; //Divide the value by 10 to get actual power value (as per manual)
        tempdouble =  (node.getResponseBuffer(0x0005) << 16) + node.getResponseBuffer(0x0004);  //get the energy value. Energy value is consists of 2 parts (2 digits of 16 bits in front and 2 digits of 16 bits at the back) and combine them to an unsigned 32bit
        PZEMEnergy = tempdouble;
        Serial.print(PZEMVoltage, 1); //Print Voltage value on Serial Monitor with 1 decimal*/
        Serial.print("V   ");
        Serial.print(PZEMCurrent, 3); Serial.print("A   ");
        Serial.print(PZEMPower, 1); Serial.print("W  ");
        Serial.print(PZEMEnergy, 0); Serial.print("Wh  ");
        Serial.println();
      } else {
        Serial.println("Failed to read modbus");
      }
      Firebase.setString(fbdo, "Kondisi/Tegangan", PZEMVoltage);
      Firebase.setString(fbdo, "Kondisi/Arus", PZEMCurrent);
      Firebase.setString(fbdo, "Kondisi/Daya", PZEMPower);
      Firebase.setString(fbdo, "Kondisi/Energi", PZEMEnergy);
      //----------------------------------------Send Humidity data to the Firebase Realtime Database.
      
      Firebase.setString(fbdo, "Kondisi/Humidity", strHum); //--> Command or code for sending Humidity data in the form of a String data type to the Firebase Realtime Database.
      //----------------------------------------Send Temperature data to the Firebase Realtime Database.
      
      Firebase.setString(fbdo, "Kondisi/Temperature", strTem); //--> Command or code for sending Temperature data in the form of a String data type to the Firebase Realtime Database.
      
      Serial.println("Sending DHT11 Sensor Data Successfully");
      Serial.println();
      
      //----------------------------------------Sending DHT11 Sensor Data, Time Data and Date Data for Data Log.
      
      String DateAndTime = currentDate + "_" + currentTime;
      Firebase.setString(fbdo, "Detail/DHT11Database" + DateAndTime + "/Date", currentDate); //--> Command or code for sending Date data in the form of a String data type to the Firebase Realtime Database.
      Firebase.setString(fbdo, "Detail/DHT11Database" + DateAndTime + "/Time", currentTime); //--> Command or code for sending Time data in the form of a String data type to the Firebase Realtime Database
      Firebase.setString(fbdo, "Detail/DHT11Database" + DateAndTime + "/_Humidity", strHum); //--> Command or code for sending Humidity data in the form of a String data type to the Firebase Realtime Database.
      Firebase.setString(fbdo, "Detail/DHT11Database" + DateAndTime + "/_Temperature", strTem); //--> Command or code for sending Temperature data in the form of a String data type to the Firebase Realtime Database.
      Firebase.setString(fbdo, "Detail/DHT11Database" + DateAndTime + "/_Water", wcond); //--> Command or code for sending Temperature data in the form of a String data type to the Firebase Realtime Database.
      Firebase.setString(fbdo, "Detail/DHT11Database" + DateAndTime + "/_Voltage", PZEMVoltage); //--> Command or code for sending Temperature data in the form of a String data type to the Firebase Realtime Database.
      Firebase.setString(fbdo, "Detail/DHT11Database" + DateAndTime + "/_Current", PZEMCurrent); //--> Command or code for sending Temperature data in the form of a String data type to the Firebase Realtime Database.
      Firebase.setString(fbdo, "Detail/DHT11Database" + DateAndTime + "/_Power", PZEMPower); //--> Command or code for sending Temperature data in the form of a String data type to the Firebase Realtime Database.
      Firebase.setString(fbdo, "Detail/DHT11Database" + DateAndTime + "/_Energy", PZEMEnergy); //--> Command or code for sending Temperature data in the form of a String data type to the Firebase Realtime Database.
      
      Serial.println("Sending Date, Time and DHT11 Sensor Data Successfully");
      Serial.println();
    }
  }
}

void setShunt(uint8_t slaveAddr) {
  static uint8_t SlaveParameter = 0x06;                                                             /* Write command code to PZEM */
  static uint16_t registerAddress = 0x0003;                                                         /* change shunt register address command code */

  uint16_t u16CRC = 0xFFFF;                                                                         /* declare CRC check 16 bits*/
  u16CRC = crc16_update(u16CRC, slaveAddr);                                                         // Calculate the crc16 over the 6bytes to be send
  u16CRC = crc16_update(u16CRC, SlaveParameter);
  u16CRC = crc16_update(u16CRC, highByte(registerAddress));
  u16CRC = crc16_update(u16CRC, lowByte(registerAddress));
  u16CRC = crc16_update(u16CRC, highByte(NewshuntAddr));
  u16CRC = crc16_update(u16CRC, lowByte(NewshuntAddr));

  Serial.println("Change shunt address");
  Serial2.write(slaveAddr); //these whole process code sequence refer to manual
  Serial2.write(SlaveParameter);
  Serial2.write(highByte(registerAddress));
  Serial2.write(lowByte(registerAddress));
  Serial2.write(highByte(NewshuntAddr));
  Serial2.write(lowByte(NewshuntAddr));
  Serial2.write(lowByte(u16CRC));
  Serial2.write(highByte(u16CRC));
  delay(10); delay(100);
  while (Serial2.available()) {
    Serial.print(char(Serial2.read()), HEX); //Prints the response and display on Serial Monitor (Serial)
    Serial.print(" ");
  }
} //setShunt Ends
