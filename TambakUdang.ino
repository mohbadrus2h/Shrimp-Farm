#include "esp_adc_cal.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <Servo.h>
#include "LiquidCrystal_I2C.h"
#include "GravityTDS.h"
#include "SimpleModbusMaster.h"
#include "time.h"

#define AERATORLOGIC DO>THRESHOLD
#define THRESHOLD 3000.0
#define SERVOUP 0
#define SERVODOWN 90

const char* ssid = "Poco";//"WS Clown Project 2.4G";
const char* password = "123456789";//"membadutbersama";
//String serverName = "http://192.168.1.106:1880/update-sensor";
String serverName = "https://api.sitesxyz.my.id/add.php";

#define lcdColumns 20
#define lcdRows 4
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);
GravityTDS gravityTds;

//====== Pinout ======//
#define PHPIN 34
#define DOPIN 35
#define TDSPIN 32
#define DS18PIN 33
#define RELAYPIN 2
#define ENPH 25
#define ENTDS 26
#define SERVOPIN 13

#define RXD2 16
#define TXD2 17

//Calibrate DO Sensor
#define CAL1_V (2352) //mv
#define CAL1_T (30)   //℃

const uint16_t DO_Table[41] = {
  14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
  11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
  9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
  7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410
};

//====== Variable ======//
int ADC[3];
float V[3];
float PH, TDS, DO, TEMP;
int button;
unsigned int VAN[2], VBN[2], VCN[2], VLN[2], AA[2], AB[2], AC[2], AAVG[2], PFTOT[2], FRQ[2];
float D_VAN, D_VBN, D_VCN, D_VLN, D_AA, D_AB, D_AC, D_AAVG, D_PFTOT, D_FRQ;

unsigned long timer, timer_website, timer_data;
unsigned int dataTiming, second, minute, hour, turnAerator;
bool dataDone = false;

const char* ntpServer = "id.pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 25200;
struct tm timeinfo;

//====== User Code Begin ======//
OneWire oneWire(DS18PIN);
DallasTemperature sensors(&oneWire);
Servo servo;

#define baud 9600
#define timeout 100
#define polling 50 // the scan rate
#define retry_count 10

// used to toggle the receive/transmit pin on the driver
#define TxEnablePin 0

enum
{
  PACKET1,
  PACKET2,
  PACKET3,
  PACKET4,
  PACKET5,
  PACKET6,
  PACKET7,
  PACKET8,
  PACKET9,
  PACKET10,
  // leave this last entry
  TOTAL_NO_OF_PACKETS
};

// Create an array of Packets for modbus_update()
Packet packets[TOTAL_NO_OF_PACKETS];
packetPointer packet1 = &packets[PACKET1];
packetPointer packet2 = &packets[PACKET2];
packetPointer packet3 = &packets[PACKET3];
packetPointer packet4 = &packets[PACKET4];
packetPointer packet5 = &packets[PACKET5];
packetPointer packet6 = &packets[PACKET6];
packetPointer packet7 = &packets[PACKET7];
packetPointer packet8 = &packets[PACKET8];
packetPointer packet9 = &packets[PACKET9];
packetPointer packet10 = &packets[PACKET10];



void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2);
  sensors.begin();
  gravityTds.begin();

  servo.attach(SERVOPIN);
  servo.write(SERVOUP);
  pinMode(RELAYPIN, OUTPUT);
  pinMode(ENTDS, OUTPUT);
  pinMode(ENPH, OUTPUT);
  digitalWrite(ENTDS, LOW);
  digitalWrite(ENPH, HIGH);

  lcd.init();
  lcd.backlight();

  WiFi.begin(ssid, password);
  lcd.setCursor(1, 0);
  lcd.print("Waiting...");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(WiFi.localIP());
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Wifi Connected!!");
  delay(5000);
  modbusSetup();
  timer_data = millis();
  lcd.clear();
}

void loop() {
  printLocalTime();

  if (minute % 2 == 0) {
    servo.write(SERVODOWN);
    if (dataTiming == 0 && second > 5) {
      takeADC();
      takeTemp();
      regressionPH();
      takeTDS();
      takeDO(V[1], TEMP);
      //Serial.println((String)"TDS = " + TDS + " || TEMP = " + TEMP + " || PH = " + PH + " || DO = " + DO);
      dataTiming = 1;
      timer_data = millis();
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("PH:");
      lcd.print(String(PH));

      lcd.setCursor(0, 1);
      lcd.print("TDS:");
      lcd.print(String(TDS));

      lcd.setCursor(0, 2);
      lcd.print("TEMP:");
      lcd.print(String(TEMP));

      lcd.setCursor(0, 3);
      lcd.print("DO:");
      lcd.print(String(DO));
    }
    else if (dataTiming == 1) {
      takeModbus();
      if (millis() - timer_data > 1000) {
        timer_data = millis();
        dataTiming = 2;
      }
    }
    else if (dataTiming == 2) {
      if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;

        String serverPath = serverName;//"?sensor1=10&sensor2=22.0&sensor3=33.0&sensor4=44.4";//&sensor5=55.5&sensor6=66.6&sensor7=77.7&sensor8=88.8&sensor9=99.0&sensor10=100&sensor11=11.1&sensor12=12.12";
        serverPath += "?sensor1=" + String(TDS);
        serverPath += "&sensor2=" + String(TEMP);
        serverPath += "&sensor3=" + String(PH);
        serverPath += "&sensor4=" + String(DO);
        serverPath += "&sensor5=" + String(D_VAN);
        serverPath += "&sensor6=" + String(D_VBN);
        serverPath += "&sensor7=" + String(D_VCN);
        serverPath += "&sensor8=" + String(D_VLN);
        serverPath += "&sensor9=" + String(D_AA);
        serverPath += "&sensor10=" + String(D_AB);
        serverPath += "&sensor11=" + String(D_AC);
        serverPath += "&sensor12=" + String(D_AAVG);
        serverPath += "&sensor13=" + String(D_PFTOT);
        serverPath += "&sensor14=" + String(D_FRQ);

        Serial.println(serverPath);

        // Your Domain name with URL path or IP address with path
        http.begin(serverPath.c_str());

        // Send HTTP GET request
        int httpResponseCode = http.GET();

        if (httpResponseCode > 0) {
          //Serial.print("HTTP Response code: ");
          //Serial.println(httpResponseCode);
          String payload = http.getString();
          //Serial.println(payload);
          if (httpResponseCode == 200) {
            button = payload.toInt();
            Serial.println(button);
          }
        }
        else {
          Serial.print("Error code: ");
          Serial.println(httpResponseCode);
        }
        // Free resources
        http.end();
        if (button == 2 && turnAerator == 0) {
          turnAerator = 1;
          if (AERATORLOGIC) digitalWrite(RELAYPIN, LOW);
          else digitalWrite(RELAYPIN, HIGH);

        }
        else if (button == 0) {
          digitalWrite(RELAYPIN, LOW);      //servo.write(0);
          turnAerator = 0;
        }
        else if (button == 1) {
          turnAerator = 0;
          digitalWrite(RELAYPIN, HIGH);
        }
      }
      else {
        WiFi.begin(ssid, password);
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print("Wifi Reconnecting...");
        Serial.println("Wifi Reconnecting...");
        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
        }
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print("Wifi Reconnected!!");
        delay(2000);
        lcd.clear();
        Serial.println("Wifi Reconnected");
      }
      timer_data = millis();
      dataTiming = 0;
    }
  }
  else {
    servo.write(SERVOUP);
    dataTiming = 0;
    turnAerator = 0;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Servo Up..");
    lcd.setCursor(0, 1);
    lcd.print(String(hour));
    lcd.print(":");
    lcd.print(String(minute));
    lcd.print(":");
    lcd.print(String(second));

    if (WiFi.status() == WL_CONNECTED) {
      HTTPClient http;
      String serverPath = "https://api.sitesxyz.my.id/btn_state.php";

      Serial.println(serverPath);
      // Your Domain name with URL path or IP address with path
      http.begin(serverPath.c_str());

      // Send HTTP GET request
      int httpResponseCode = http.GET();

      if (httpResponseCode > 0) {
        //Serial.print("HTTP Response code: ");
        //Serial.println(httpResponseCode);
        String payload = http.getString();
        //Serial.println(payload);
        if (httpResponseCode == 200) {
          button = payload.toInt();
          Serial.println(button);
        }
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
      if (button == 0) {
        digitalWrite(RELAYPIN, LOW);      //servo.write(0);
        turnAerator = 0;
      }
      else if (button == 1) {
        turnAerator = 0;
        digitalWrite(RELAYPIN, HIGH);
      }
    }
    else {
      WiFi.begin(ssid, password);
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Wifi Reconnecting...");
      Serial.println("Wifi Reconnecting...");
      while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
      }
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Wifi Reconnected!!");
      delay(2000);
      lcd.clear();
      Serial.println("Wifi Reconnected");
    }
  }

}

float f_2uint_float(unsigned int uint1, unsigned int uint2) {    // reconstruct the float from 2 unsigned integers
  union f_2uint {
    float f;
    uint16_t i[2];
  };
  union f_2uint f_number;
  f_number.i[0] = uint1;
  f_number.i[1] = uint2;
  return f_number.f;
}

void takeADC() {
  digitalWrite(ENPH, HIGH);
  digitalWrite(ENTDS, LOW);
  ADC[0] = analogRead(PHPIN);
  digitalWrite(ENPH, LOW);
  digitalWrite(ENTDS, HIGH);
  delay(200);
  ADC[2] = analogRead(TDSPIN);
  digitalWrite(ENPH, HIGH);
  digitalWrite(ENTDS, LOW);

  ADC[1] = analogRead(DOPIN);
  //  V[0] = readADC_Cal(ADC[0]);
  //  V[1] = readADC_Cal(ADC[1]);
  //  V[2] = readADC_Cal(ADC[2]);
  V[0] = (ADC[0] / 4095.0) * 3300.0;
  V[1] = (ADC[1] / 4095.0) * 3300.0;
  V[2] = (ADC[2] / 4095.0) * 3300.0;
  //Serial.println((String)"ADC1 = " + ADC[0] + " || ADC2 = " + ADC[1] + " || ADC3 = " + ADC[2]);
  //Serial.println((String)"V1 = " + V[0] + " || V2 = " + V[1] + " || V3 = " + V[2]);
  //Serial.println(ADC[0]);
}

void regressionPH() {
  float bufferPH = -0.00311 * ADC[0] + 16.46214;
  if (bufferPH < 10.0 && bufferPH > 3.0) {
    PH = bufferPH;
  }
}

void takeTDS() {
  gravityTds.setTemperature(TEMP);  // set the temperature and execute temperature compensation
  gravityTds.update();  //sample and calculate
  TDS = gravityTds.getTdsValue();  // then get the value
}

void takeTemp() {
  sensors.requestTemperatures();
  TEMP = sensors.getTempCByIndex(0);
}

void takeDO(uint32_t voltage_mv, int temperature_c)
{
  uint16_t V_saturation = (uint32_t)CAL1_V + (uint32_t)35 * temperature_c - (uint32_t)CAL1_T * 35;
  DO = (voltage_mv * DO_Table[temperature_c] / V_saturation);
}

uint32_t readADC_Cal(int ADC_Raw)
{
  esp_adc_cal_characteristics_t adc_chars;

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

void takeModbus() {
  // = modbus_update(packets);
  for (int i = 0; i < 10; i++) {
    unsigned int connection_status = modbus_update(packets);
    if (connection_status != TOTAL_NO_OF_PACKETS) {
      Serial.println("MODBUS FAILED!!!");
    }
  }


  //long newTimer = millis();
  //if (newTimer -  timer >= 1000) {
  D_VAN = f_2uint_float(VAN[1], VAN[0]);
  D_VBN = f_2uint_float(VBN[1], VBN[0]);
  D_VCN = f_2uint_float(VCN[1], VCN[0]);
  D_VLN = f_2uint_float(VLN[1], VLN[0]);

  D_AA = f_2uint_float(AA[1], AA[0]);
  D_AB = f_2uint_float(AB[1], AB[0]);
  D_AC = f_2uint_float(AC[1], AC[0]);
  D_AAVG = f_2uint_float(AAVG[1], AAVG[0]);

  D_PFTOT = f_2uint_float(PFTOT[1], PFTOT[0]);
  D_FRQ = f_2uint_float(FRQ[1], FRQ[0]);

  Serial.println((String)
                 "VAN = " + D_VAN +
                 " || VBN = " + D_VBN +
                 " || VCN = " + D_VCN +
                 " || VLN = " + D_VLN +
                 " || AA = " + D_AA +
                 " || AB = " + D_AB +
                 " || AC = " + D_AC +
                 " || AAVG = " + D_AAVG +
                 " || PF = " + D_PFTOT +
                 " || FRQ = " + D_FRQ);

  //Serial.println((String)PFTOT[1] + "  " + PFTOT[0]);
  //timer = newTimer;
  //}
}

void modbusSetup() {
  packet1->id = packet2->id = packet3->id = packet4->id = packet5->id =
                                packet6->id = packet7->id = packet8->id = packet9->id = packet10->id = 1;
  packet1->function = packet2->function = packet3->function = packet4->function = packet5->function =
      packet6->function = packet7->function = packet8->function = packet9->function = packet10->function = READ_HOLDING_REGISTERS;
  packet1->no_of_registers = packet2->no_of_registers = packet3->no_of_registers = packet4->no_of_registers = packet5->no_of_registers =
                               packet6->no_of_registers = packet7->no_of_registers = packet8->no_of_registers = packet9->no_of_registers = packet10->no_of_registers = 2;

  packet1->address = 3027;
  packet1->register_array = VAN;
  packet2->address = 3029;
  packet2->register_array = VBN;
  packet3->address = 3031;
  packet3->register_array = VCN;
  packet4->address = 3035;
  packet4->register_array = VLN;

  packet5->address = 2999;
  packet5->register_array = AA;
  packet6->address = 3001;
  packet6->register_array = AB;
  packet7->address = 3003;
  packet7->register_array = AC;
  packet8->address = 3009;
  packet8->register_array = AAVG;

  packet9->address = 3191;
  packet9->register_array = PFTOT;
  packet10->address = 3109;
  packet10->register_array = FRQ;

  modbus_configure(baud, timeout, polling, retry_count, TxEnablePin, packets, TOTAL_NO_OF_PACKETS);

  timer = millis();
}

void printLocalTime() {
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  second = timeinfo.tm_sec;
  minute = timeinfo.tm_min;
  hour = timeinfo.tm_hour;
  //Serial.println(hour);
}
