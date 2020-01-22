#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "SPIFFS.h"
#include <Bounce2.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include "Adafruit_miniTFTWing.h"

Adafruit_miniTFTWing ss;
#define TFT_RST    -1    // we use the seesaw for resetting to save a pin
#ifdef ESP32
   #define TFT_CS   14
   #define TFT_DC   32
#endif

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *myMotor[4] = {
  AFMS.getMotor(1),
  AFMS.getMotor(2),
  AFMS.getMotor(3),
  AFMS.getMotor(4),
};
 
const char* ssid = "yourssid";
const char* password =  "testpassword";

AsyncWebServer server(80);

int s_motorState, s_direction, s_speed;
int motorState, direction, speed, m1, m2, m3, m4;
bool irSensors, tiltSensor, ultrasonicSensor, headlights = true;
bool fault, faultInvertDisplay, headlightsBlink;
unsigned long commandStart, faultStart, hudRefreshStart, headlightsStart, headlightsBlinkStart;
unsigned long ultrasonicInches;
const unsigned long commandExpire = 10000;

Bounce debouncedTiltSensor = Bounce();
#define TILT_PIN 26 // A0

#define TRIGGER_PIN 25 // A1
#define ECHO_PIN 34 // A2 (non-output)

#define IR_RIGHT_PIN 36 // A4 (non-output)
#define IR_LEFT_PIN 39 // A3 (non-output)

#define BATTERY_PIN A9

#define HEADLIGHTS_PIN 21
 
void setup(){
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Infrared sensors
  pinMode(IR_RIGHT_PIN, INPUT);
  pinMode(IR_LEFT_PIN, INPUT);
  irSensors = true;

  // Tilt sensor
  pinMode(TILT_PIN, INPUT);
  debouncedTiltSensor.attach(TILT_PIN);
  debouncedTiltSensor.interval(100); // interval in ms
  tiltSensor = true;

  // Ultrasonic sensor
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // SPI file system
  if(!SPIFFS.begin(true)){
    Serial.println("An error has occurred while mounting SPIFFS");
  }

  // Onboard LCD
  if (!ss.begin()) {
    Serial.println("seesaw init error!");
    while(1);
  }
  else Serial.println("seesaw started");

  ss.tftReset();
  ss.setBacklight(0x0); //set the backlight fully on

  tft.initR(INITR_MINI160x80);   // initialize a ST7735S chip, mini display

  tft.setRotation(3);
  tft.fillScreen(ST77XX_BLACK);
  
  tft.fillRect(0, 0, tft.width(), 14, ST77XX_MAGENTA);
  tft.fillRect(0, tft.height()-14, tft.width(), tft.height(), ST77XX_MAGENTA);

  tft.setCursor(0, 18);    
  tft.setTextSize(1);
  tft.setTextWrap(false);

  Serial.println("LCD Initialized");
 
  // Access point, default 192.168.4.1 ---------
  WiFi.mode(WIFI_AP);
  
  Serial.println("Creating Accesspoint");

  WiFi.softAP("DK_Rover", "testpassword");
  Serial.print("AP IP address:\t");
  Serial.println(WiFi.softAPIP());
    
  tft.print(F("Created Soft AP:"));
  tft.println(WiFi.softAPIP().toString());
  // ------------------------------------------

  unsigned long wifiStart = millis();
  WiFi.begin(ssid, password);
 
  Serial.print(F("Connecting to "));
  tft.print(F("Connecting to "));
  Serial.print(ssid);
  tft.print(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
    tft.print(F("."));
    if(millis() - wifiStart > 8000) break;
  }
 
  Serial.println();
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

  tft.println();
  tft.print(F("Local IP:"));
  tft.print(WiFi.localIP().toString());

  // Set up ESP webserver endpoints
  server.on("/drive", HTTP_GET, [](AsyncWebServerRequest *request){
    digitalWrite(LED_BUILTIN, HIGH);
 
    int paramsNr = request->params();
 
    for(int i=0;i<paramsNr;i++){
 
        AsyncWebParameter* p = request->getParam(i);

        if(p->name() == "motorState") {
          s_motorState = p->value().toInt();
        }
        if(p->name() == "direction") {
          s_direction = p->value().toInt();
        }
        if(p->name() == "speed") {
          s_speed = p->value().toInt();
        }
        if(p->name() == "m1") {
          m1 = p->value().toInt();
        }
        if(p->name() == "m2") {
          m2 = p->value().toInt();
        }
        if(p->name() == "m3") {
          m3 = p->value().toInt();
        }
        if(p->name() == "m4") {
          m4 = p->value().toInt();
        }   
        if(p->name() == "ir") {
          irSensors = p->value() == "true";
        } 
        if(p->name() == "tilt") {
          tiltSensor = p->value() == "true";
        }  
        if(p->name() == "ultrasonic") {
          ultrasonicSensor = p->value() == "true";
        }
        if(p->name() == "headlights") {
          if(headlights != (p->value() == "true") && !headlights) {
            headlightsBlinkStart = headlightsStart = millis();
          }
          headlights = p->value() == "true";
        }                    
    }

    AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", "Command Received");
    
    // Allow cross-origin requests coming from website with rich UI
    response->addHeader("Access-Control-Allow-Origin","*");
    request->send(response);

    digitalWrite(LED_BUILTIN, LOW);

    if (ON_STA_FILTER(request)) {
      // Request through router
      Serial.print("Drive request from remote IP to: ");
      Serial.println(WiFi.localIP().toString());
    } else {
      // Request through direct connection
      Serial.print("Drive request from client IP to: ");
      Serial.println(WiFi.softAPIP().toString());
    }

    commandStart = faultStart = millis();

    printToTFT(F(""));
    if(s_motorState == RELEASE) {
      tft.fillRoundRect(96, 18, 20, 45, 5, ST77XX_RED);
      tft.fillRoundRect(127, 18, 20, 45, 5, ST77XX_RED);
    } else {
      tft.fillTriangle(97, 25, 97, 55, 135, 40, ST77XX_GREEN);
    }
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/www/ui.html", String(), false, processor);
    Serial.println("Sending web UI page");
    printToTFT(F("Sending UI"));
  });

  server.serveStatic("/", SPIFFS, "/www/");

  server.onNotFound([](AsyncWebServerRequest *request){
    request->send(404, "text/plain", "404: Page not found");
    Serial.println("Sending 404 error");
    printToTFT(F("Sending 404"));
  });
 
  server.begin();

  AFMS.begin();  // Start motors with the default frequency 1.6KHz
  s_motorState = RELEASE;  

  tft.println();
  tft.setTextSize(2);
  if(WiFi.localIP().toString() != "0.0.0.0") {
    tft.println("Ready");
  }
  else {
    tft.println("Connect to me");

  }

  pinMode(HEADLIGHTS_PIN, OUTPUT);
  digitalWrite(HEADLIGHTS_PIN, HIGH);
  headlightsStart = millis();
}
 
void loop(){

  if(irSensors && digitalRead(IR_RIGHT_PIN) == LOW && motorState == FORWARD) {
    Serial.println("Front right object detected");
    printToTFT(F("IR Right Trig"));
    s_motorState = RELEASE;
    fault = true;
  }
  if(irSensors && digitalRead(IR_LEFT_PIN) == LOW && motorState == FORWARD) {
    Serial.println("Front left object detected"); // maxlength
    printToTFT(F("IR Left Trig"));
    s_motorState = RELEASE;
    fault = true;
  }

  debouncedTiltSensor.update();
  if(tiltSensor && debouncedTiltSensor.read() == LOW && motorState == FORWARD) {
    Serial.println("Tilt sensor triggered");
    printToTFT(F("Tilt Sensor"));
    s_motorState = RELEASE;
    fault = true;
  }

  if(motorState != s_motorState || speed != s_speed || direction != s_direction) {
    motorState = s_motorState; 
    speed = s_speed; 
    direction = s_direction;

    myMotor[0]->run(motorState);
    myMotor[1]->run(motorState);
    myMotor[2]->run(motorState);
    myMotor[3]->run(motorState); 

    if(motorState == FORWARD) { // set speed with power adjustment based on testing to go straight
      myMotor[0]->setSpeed((speed - (direction > 0 ? direction : 0)) * 10 + (((speed > 15 ? -2 : -1) * speed)/2) + ((m1 * speed)/2));
      myMotor[1]->setSpeed((speed + (direction < 0 ? direction : 0)) * 10 + ((m2 * speed)/2));
      myMotor[2]->setSpeed((speed + (direction < 0 ? direction : 0)) * 10 + ((m3 * speed)/2));
      myMotor[3]->setSpeed((speed - (direction > 0 ? direction : 0)) * 10 + (((speed > 15 ? -2 : -1) * speed)/2) + ((m4 * speed)/2));
      fault = false;
    } else if(motorState == BACKWARD) { // backwards always goes straight
      myMotor[0]->setSpeed(speed * 7 + ((m1 * speed)/2));
      myMotor[1]->setSpeed(speed * 7 + (((speed > 15 ? -2 : -1) * speed)/2) + ((m2 * speed)/2));
      myMotor[2]->setSpeed(speed * 7 + (((speed > 15 ? -2 : -1) * speed)/2) + ((m3 * speed)/2));
      myMotor[3]->setSpeed(speed * 7 + ((m4 * speed)/2)); 
      fault = false; 
    }
    tft.invertDisplay(fault);
  }

  // Blink display on sensor tripped fault
  if(fault) {
    if(millis() - faultStart > 500) {
        tft.invertDisplay(faultInvertDisplay);
        faultInvertDisplay = !faultInvertDisplay;
        faultStart = millis();
    }
  } else {
    // If no command received after timeout period, stop
    if (millis() - commandStart > commandExpire && (motorState == FORWARD || motorState == BACKWARD))
    {
      s_motorState = RELEASE;
      commandStart = millis();
      digitalWrite(LED_BUILTIN, HIGH); // out of range / no command in 10 secs
      printToTFT(F("What next?"));
    }
  }

  // Blink headlights
  if(millis() - headlightsStart < 2500) {
    if(millis() - headlightsBlinkStart > (millis() - headlightsStart) / 10) {
      headlightsBlink = !headlightsBlink;
      digitalWrite(HEADLIGHTS_PIN, headlightsBlink);
      headlightsBlinkStart = millis();
    }
  } else {
      digitalWrite(HEADLIGHTS_PIN, headlights);    
  }

  // Update top line HUD stats
  if(millis() - hudRefreshStart > 200) {

    // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
    // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
  
    // Read the signal from the sensor: a HIGH pulse whose
    // duration is the time (in microseconds) from the sending
    // of the ping to the reception of its echo off of an object.
    ultrasonicInches = pulseIn(ECHO_PIN, HIGH) * 0.00675;

    tft.fillRect(0, 0, tft.width(), 14, ST77XX_MAGENTA);
    tft.setCursor(0, 2);    
    tft.setTextSize(1);
    tft.print("Batt:");
 
    float measuredvbat = analogRead(BATTERY_PIN);
    measuredvbat *= 3.45;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    // total voltage muliplier is 11.385 ---DK
    measuredvbat /= 1024; // convert to voltage
    tft.print(measuredvbat);

    tft.print(" Pwr:");
    tft.print(0.0);    
    tft.print(" Dist:");
    if(ultrasonicSensor) tft.print(ultrasonicInches); else tft.print("off");
    tft.print("\"");   

    if(ultrasonicSensor && motorState == FORWARD) {
      if(ultrasonicInches < speed) {
        if(ultrasonicInches > 2 && speed > 0) {
          s_speed = ultrasonicInches;
          printToTFT(F("Distance: "), String(s_speed));
          if(ultrasonicInches < 6) { // 3sec failsafe timeout when getting close
            commandStart = millis() - commandExpire + 3000;
          }
        } else {
          headlightsBlinkStart = headlightsStart = millis();
          s_motorState = RELEASE;
          printToTFT(F("I'm stuck"));
        }
      }
    }

    hudRefreshStart = millis();
  }
}

// Web template processor
String processor(const String& var)
{
  if(var == "AP_IP_ADDRESS")
    return WiFi.softAPIP().toString();
  return String();
}

void printToTFT(const String& var)
{
  printToTFT(var, "");
}

void printToTFT(const String& line1, const String& line2)
{
  // height-28?? doesn't make sense but it came from testing
  tft.fillRect(0, 14, tft.width(), tft.height()-28, ST77XX_BLACK);
  tft.setCursor(0, 18);    
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE);
  tft.println(line1);
  tft.println(line2);
}
