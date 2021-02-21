/*********
  Bill Poulsen
  www.iot-techshop.com
*********/

// Required libraries
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include <Wire.h> //I2C
#include <BME280I2C.h> //
#include "SparkFun_MMA8452Q.h"
MMA8452Q accel;


BME280I2C bme;    // Default : addr 0x76 forced mode, standby time = 1000 ms
// I/O For CUSPIX V1 with ESP32(You can set up your own GPIO Pin Assignements
#define SEN_PWR_CTL 15 //Controls 3.3V Power to All Sensors  High = ON/Low = OFF//Only for Cuspix TiDi ESP32 Board
#define LED_RED 2 //On Board RED LED   


// Replace with your network credentials
const char* ssid = "your ssid";
const char* password = "your wifi password";

// Set LED GPIO
const int ledPin = 2;
// Stores LED state
String ledState;
String myID;
String myMac;

const char* PARAM_INPUT_1 = "output";
const char* PARAM_INPUT_2 = "state";

float myTemp;
float myHumid;
float myPress;
float tmpOffset = 0.0;//Offset needed on some boards where BME280 is near warm components
float temp(NAN), hum(NAN), pres(NAN);

int Taps = 0;

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

uint8_t readline(char *buff, uint8_t maxbuff, uint16_t timeout = 0);
uint8_t type;
char replybuffer[255]; // this is a large buffer for replies
char imei[16] = {0}; // MUST use a 16 character buffer for IMEI!

String accPos = "";
uint16_t year;
uint8_t month, day, hour, minute;

int value = 0;

void setup() {
  // Serial port for debugging purposes
  Wire.begin();
  bme.begin();
  accel.begin();
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  digitalWrite(SEN_PWR_CTL, HIGH);
  pinMode(SEN_PWR_CTL, OUTPUT);
  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    return;
  }
  myID = getEspID();
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi..");
  }

  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
  printBME280Data();

  Serial.println(readDHTTemperature());
  Serial.println(readDHTHumidity());


  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Route for second / web page
  server.on("/index2.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index2.html", String(), false, processor);
  });
  // Route for second / web page
  server.on("/index3.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index3.html", String(), false, processor);
  });

  // Route for second / web page
  server.on("/index4.html", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/index4.html", String(), false, processor);
  });
  // Route to load style.css file
  server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SPIFFS, "/style.css", "text/css");
  });

  // Route to set GPIO to HIGH
  server.on("/on", HTTP_GET, [](AsyncWebServerRequest * request) {
    digitalWrite(ledPin, HIGH);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  // Route to set GPIO to LOW
  server.on("/off", HTTP_GET, [](AsyncWebServerRequest * request) {
    digitalWrite(ledPin, LOW);
    request->send(SPIFFS, "/index.html", String(), false, processor);
  });

  server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readDHTTemperature().c_str());
  });

  server.on("/humidity", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readDHTHumidity().c_str());
  });
  server.on("/pressure", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", readBMEPress().c_str());
  });
  server.on("/position", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getAccel().c_str());
  });
  server.on("/taps", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getTaps().c_str());
  });
  server.on("/gforcex", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getGx().c_str());

  });

  server.on("/gforcey", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getGy().c_str());

  });
  server.on("/gforcez", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", getGz().c_str());

  });


  server.on("/tapreset", HTTP_GET, [](AsyncWebServerRequest * request) {
    Taps = 0;
    request->send(SPIFFS, "/index4.html", String(), false, processor);
  });


  // Send a GET request to <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest * request) {
    String inputMessage1;
    String inputMessage2;
    // GET input1 value on <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) {
      inputMessage1 = request->getParam(PARAM_INPUT_1)->value();
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
      digitalWrite(inputMessage1.toInt(), inputMessage2.toInt());
    }
    else {
      inputMessage1 = "No message sent";
      inputMessage2 = "No message sent";
    }
    Serial.print("GPIO: ");
    Serial.print(inputMessage1);
    Serial.print(" - Set to: ");
    Serial.println(inputMessage2);
    request->send(200, "text/plain", "OK");
  });


  // Start server
  server.begin();

  Serial.println(getAccel());
}

void loop() {

}


String idDisplay(const String& var) {
  if (var == "devID") {
    Serial.println(myID);
  }
  return myID;
}

String readDHTTemperature() {

  BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  float t = temp + tmpOffset;

  Serial.println(t);
  return String(t);

}
String readDHTHumidity() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

  BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  //float h = dht.readHumidity();
  bme.read(pres, temp, hum, tempUnit, presUnit);
  float h = hum;
  return String(h);
}
String readBMEPress() {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)

  BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);
  bme.read(pres, temp, hum, tempUnit, presUnit);
  float p = pres * 0.0002953;
  return String(p);
}

String processor(const String& var) {
  Serial.println(var);
  if (var == "STATE") {
    if (digitalRead(ledPin)) {
      ledState = "ON";
    }
    else {
      ledState = "OFF";
    }
    Serial.print(ledState);
    return ledState;
  }
  else if (var == "IDDEVICE") {
    return getEspID();
  }
  else if (var == "TEMPERATURE") {
    return readDHTTemperature();
  }
  else if (var == "HUMIDITY") {
    return readDHTHumidity();
  }
  else if (var == "PRESSURE") {
    return readBMEPress();
  }
  else if (var == "POSITION") {
    return getAccel();
  }
  else if (var == "GFORCEX") {
    return getGx();
  }
  else if (var == "GFORCEY") {
    return getGy();
  }
  else if (var == "GFORCEZ") {
    return getGz();
  }
  else if (var == "TAPS") {
    return getTaps();
  }

  //Serial.println(var);
  if (var == "BUTTONPLACEHOLDER") {
    String buttons = "";
    buttons += "<h4>Output - LED RED</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"2\" " + outputState(2) + "><span class=\"slider\"></span></label>";
    buttons += "<h4>Output - SEN PWR</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"15\" " + outputState(15) + "><span class=\"slider\"></span></label>";
    buttons += "<h4>Output - LTE PWR</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"26\" " + outputState(26) + "><span class=\"slider\"></span></label>";
    return buttons;
  }

  return String();
}

String outputState(int output) {
  if (digitalRead(output)) {
    return "checked";
  }
  else {
    return "";
  }
}








String getEspID() {
  myMac = WiFi.macAddress();
  //Creates device ID from ESP01 MAC address
  myID = myMac.substring(0, 2) +  myMac.substring(3, 5) +  myMac.substring(6, 8) +  myMac.substring(9, 11) +  myMac.substring(12, 14) +  myMac.substring(15, 17);

  return myMac.substring(0, 2) +  myMac.substring(3, 5) +  myMac.substring(6, 8) +  myMac.substring(9, 11) +  myMac.substring(12, 14) +  myMac.substring(15, 17);

}

void printBME280Data()

{
  //float temp(NAN), hum(NAN), pres(NAN);

  BME280::TempUnit tempUnit(BME280::TempUnit_Fahrenheit);
  BME280::PresUnit presUnit(BME280::PresUnit_Pa);

  bme.read(pres, temp, hum, tempUnit, presUnit);
  Serial.print("Temp:"); Serial.print(temp); Serial.print("*F Humid:"); Serial.print(hum); Serial.print("% RH Pres:"); Serial.print(pres); Serial.println(" Pa");
  myTemp = temp + tmpOffset;
  myHumid = hum;
  myPress = pres;

  Serial.println(myTemp);
  Serial.println(myHumid);
  Serial.println(myPress);
  delay(100);
}



String getAccel() {

  String accPos = "";
  Serial.println("START");


  if (accel.isRight() == true) {
    accPos = "Right";
  }
  else if (accel.isLeft() == true) {
    accPos = "Left";
    Serial.println("Left");
  }
  else if (accel.isUp() == true) {
    accPos = "Up";
  }
  else if (accel.isDown() == true) {
    accPos = "Down";
  }
  else if (accel.isFlat() == true) {
    accPos = "Flat";
  }

  Serial.println(accPos);

  return accPos;
  Serial.println("END");

}

String getGx() {
  String gX;
  if (accel.available()) {      // Wait for new data from accelerometer
    // Acceleration of x, y, and z directions in g units
    gX = String(accel.getCalculatedX(), 2);
    Serial.println(gX);
  }
  return gX;
}
String getGy() {
  String gY;
  if (accel.available()) {      // Wait for new data from accelerometer
    // Acceleration of x, y, and z directions in g units
    gY = String(accel.getCalculatedY(), 2);
  }
  return gY;

}
String getGz() {
  String gZ;
  if (accel.available()) {
    // Wait for new data from accelerometer
    // Acceleration of x, y, and z directions in g units
    gZ = String(accel.getCalculatedZ(), 2);
  }
  return gZ;

}
String getTaps() {
  if (accel.available()) {      // Wait for new data from accelerometer
    // Prints "Tap" each time accelerometer detects a tap

    if (accel.readTap() > 0) {
      Taps = Taps + 1;

      Serial.println(Taps);
    }
  }

  return String(Taps);
}
