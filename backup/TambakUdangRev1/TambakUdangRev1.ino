#include "esp_adc_cal.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include <WiFi.h>
#include "LiquidCrystal_I2C.h"
#include "GravityTDS.h"

const char* ssid = "WS Clown Project 2.4G";
const char* password = "membadutbersama";
//String serverName = "http://192.168.1.106:1880/update-sensor";
String serverName = "https://api.sitesxyz.my.id/add.php";

#define lcdColumns 16
#define lcdRows 2
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);  
GravityTDS gravityTds;

//====== Pinout ======//
#define PHPIN 34
#define DOPIN 35
#define TDSPIN 32
#define DS18PIN 33
#define RELAYPIN 2

#define RXD2 16
#define TXD2 17  

//Calibrate DO Sensor
#define CAL1_V (2352) //mv
#define CAL1_T (30)   //℃

const uint16_t DO_Table[41] = {
    14460, 14220, 13820, 13440, 13090, 12740, 12420, 12110, 11810, 11530,
    11260, 11010, 10770, 10530, 10300, 10080, 9860, 9660, 9460, 9270,
    9080, 8900, 8730, 8570, 8410, 8250, 8110, 7960, 7820, 7690,
    7560, 7430, 7300, 7180, 7070, 6950, 6840, 6730, 6630, 6530, 6410};
    
//====== Variable ======//
int ADC[3];
float V[3];
float PH,TDS,DO,TEMP;
int button;

//====== User Code Begin ======//
OneWire oneWire(DS18PIN);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD2, TXD2);
  sensors.begin();
  gravityTds.begin();

  pinMode(RELAYPIN,OUTPUT);
  
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
  lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Wifi Connected!!");
  delay(2000);
}

void loop() {
  takeADC();
  takeTemp();
  regressionPH();
  takeTDS();
  takeDO(V[1],TEMP);
  Serial.println((String)"TDS = " + TDS + " || TEMP = " + TEMP + " || PH = " + PH + " || DO = " + DO);

  /*
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;

    String serverPath = serverName + "?sensor1=10&sensor2=22.0&sensor3=33.0&sensor4=44.4";//&sensor5=55.5&sensor6=66.6&sensor7=77.7&sensor8=88.8&sensor9=99.0&sensor10=100&sensor11=11.1&sensor12=12.12";

    // Your Domain name with URL path or IP address with path
    http.begin(serverPath.c_str());

    // Send HTTP GET request
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      //Serial.print("HTTP Response code: ");
      //Serial.println(httpResponseCode);
      String payload = http.getString();
      //Serial.println(payload);
      if (httpResponseCode == 200){
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
  }
  else {
    WiFi.begin(ssid, password);
    Serial.println("Wifi Reconnecting...");
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("Wifi Reconnected");
  }
  */
}

void takeADC() {
  ADC[0] = analogRead(PHPIN);
  ADC[1] = analogRead(DOPIN);
  ADC[2] = analogRead(TDSPIN);
  //  V[0] = readADC_Cal(ADC[0]);
  //  V[1] = readADC_Cal(ADC[1]);
  //  V[2] = readADC_Cal(ADC[2]);
  V[0] = (ADC[0] / 4095.0) * 3300.0;
  V[1] = (ADC[1] / 4095.0) * 3300.0;
  V[2] = (ADC[2] / 4095.0) * 3300.0;
  Serial.println((String)"ADC1 = " + ADC[0] + " || ADC2 = " + ADC[1] + " || ADC3 = " + ADC[2]);
  //Serial.println((String)"V1 = " + V[0] + " || V2 = " + V[1] + " || V3 = " + V[2]);
  //Serial.println(ADC[0]);
}

void regressionPH(){
  PH = -0.00311 * ADC[0] + 16.46214;
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
