// --- 1. Libraries ---
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <time.h>
#include <U8g2lib.h>
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp>
#include "ac_LG.hpp"
#include <Adafruit_AM2320.h>


// --- 2. Hardware Objects ---
AsyncWebServer server(80);
Aircondition_LG MyLG_Aircondition;
U8G2_SH1107_SEEED_128X128_F_HW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);
Adafruit_AM2320 am2320 = Adafruit_AM2320();

// --- 3. Network & Time Constants ---
const char* ssid = "ShOffice_2_4GHz";
const char* password = "workworkwork";
//const char* ssid = "HomeWifi";
//const char* password = "Amelia2016George2018";
const char* TZ_INFO = "GMT0BST,M3.5.0/1,M10.5.0";
const char* ntpServer = "pool.ntp.org";

// --- 4. System State Variables ---
int TargetTemperature = 22;              //default temperature set point
int AC_Turn_On_Time_Hour = 5;            //default start time hour i.e. 5am
int AC_Turn_On_Time_Minute = 0;          //default start time minutes i.e. 00 past the hour
const int AC_Turn_Off_Time_Hour = 21;    //power off time for AC
const int AC_Turn_Off_Time_Minute = 00;  //power off time for AC
bool runSystem = 0;
bool TimeSyncSuccessBool = false;
bool AC_State_Previous = 0;
unsigned long previousWifiCheckMillis = 0;
int myModeParameter = 0;
int myTemperatureParameter = 0;
int myCommand = 0;
int myParameter = 0;
String SystemState = "OFF";
float currentTemp = 0;    // Global to store the reading
float currentHum = 0;

// --- 5. Parameters & Web Input Storage ---
char myParameter1[4] = "xx";  // Hour, use default from above to initialise
char myParameter2[4] = "xx";  // Minute, use default from above to initialise
char myParameter3[4] = "22";  // Temp, use default from above to initialise
char myParameter4[4] = "xx";  // Mode, set to xx to show it has not been initialised
char myParameter5[4] = "xx";  // Run, set to xx to show it has not been initialised

// --- 6. Timing & IR Sequence ---
struct tm timenow;                                  //time structure for getting network time and setting time within the code
unsigned long TimeIntervalPrevious = 0;             //used for periodic time sync with the network ntp
const int TimeSyncInterval = (1000 * 60 * 60 * 6);  // 6 Hours

//IR Command timing and intervals
unsigned long previousIRMillis = 0;  // Time since previous IR commands was sent
const long IRInterval = 2000;        // interval between sending successive IR commands
int IRCommandStep = 0;               // used to enable triggers IR commands and know if we are currently sending them

//OLED interval timer
unsigned long OLEDUpdateIntervalPrevious = 0;  //how long since our previous update to the OLED
const int OLEDUpdateInterval = 500;

//timer interval for checking wifi status
const long wifiCheckInterval = 10000;  // Check every 10 seconds


// --- 7. OLED Variables ---
char DisplayTimeWeekDay[10];
char DisplayTimeHour[3];
char DisplayTimeMinute[3];
char DisplayMidnightMinutes[5];
char DisplayTemperature[3];
char DisplayHumidity[3];
char displayStartTimeHour[2] = { 'h', 'h' };
char displayStartTimeMinute[2] = { 'm', 'm' };
String AC_Mode_State = "null";
String Wifi_State = "unknown";
String RunSystemStr = "INITIALISED";
String CurrentState = "INITIALISED";

// --- 8. WebPage Presentation converter ---
String processor(const String& var) {
  if (var == "HOUR") return String(myParameter1);
  if (var == "MIN") return String(myParameter2);
  if (var == "TEMP") return String(myParameter3);
  if (var == "MODE") return String(myParameter4);
  if (var == "RUN") return String(myParameter5);
  return String();
}

//web page that can handle multiple inputs at the same time
// Change the variable name to index_html_template to avoid confusion
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>AC Controller</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; text-align: center; margin-top: 30px; }
    form { display: inline-block; text-align: left; background: #f2f2f2; padding: 20px; border-radius: 10px; margin-bottom: 20px; }
    .row { margin: 10px 0; }
    label { display: inline-block; width: 160px; font-weight: bold; }
    .trigger-btn { background-color: #ff4c4c; color: white; border: none; padding: 15px; border-radius: 5px; cursor: pointer; width: 100%; font-weight: bold; }
    input[type="submit"] { width: 100%; height: 30px; }
  </style>
</head><body>
  <h2>AC Configuration</h2>
  <form action="/get">
    <div class="row"><label>Hour Start:</label><input type="text" name="input1" placeholder="%HOUR%"></div>
    <div class="row"><label>Minute Start:</label><input type="text" name="input2" placeholder="%MIN%"></div>
    <div class="row"><label>Temp Setpoint:</label><input type="text" name="input3" placeholder="%TEMP%"></div>
    <div class="row"><label>Mode (h/c):</label><input type="text" name="input4" placeholder="%MODE%"></div>
    <div class="row"><label>System Run (1/0):</label><input type="text" name="input5" placeholder="%RUN%"></div>
    <input type="submit" value="Update Settings">
  </form>

  <br>

  <form action="/trigger">
    <button type="submit" class="trigger-btn">SEND IR COMMANDS NOW</button>
  </form>
  <p><a href="/">Refresh Status</a></p>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest* request) {
  request->send(404, "text/plain", "Not found");
}

void setup() {
  Serial.begin(115200);

  sprintf(myParameter1, "%02d", AC_Turn_On_Time_Hour);
  sprintf(myParameter2, "%02d", AC_Turn_On_Time_Minute);

  // 1. REGISTER THE EVENT HANDLER
  WiFi.onEvent(WiFiEvent);  // This tells the ESP32 to run WiFiEvent() on network changes

  // 2. INITIAL CONNECTION
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  // This starts the connection process, but does NOT wait.
  // The ARDUINO_EVENT_WIFI_STA_GOT_IP event will handle the rest.
  WiFi.setTxPower(WIFI_POWER_8_5dBm);  // Lower power often leads to a more stable connection on C3
  WiFi.begin(ssid, password);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/trigger", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (IRCommandStep == 0) {  // Only start if not already sending
      IRCommandStep = 1;
      previousIRMillis = millis();  // Initialize the non-blocking timer
      Serial.println("Manual IR Triggered via Web");
      request->send(200, "text/html", "IR Sequence Started!<br><a href=\"/\">Return</a>");
    } else {
      request->send(200, "text/html", "Sequence already in progress.<br><a href=\"/\">Return</a>");
    }
  });

  // FIX 2: Update /get to handle multiple parameters
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest* request) {
    // Check each parameter independently (no 'else if')
    // Only update if the parameter exists AND has a value (length > 0)
    if (request->hasParam("input1") && request->getParam("input1")->value().length() > 0) {
      strncpy(myParameter1, request->getParam("input1")->value().c_str(), sizeof(myParameter1) - 1);
      myParameter1[sizeof(myParameter1) - 1] = '\0';
    }
    if (request->hasParam("input2") && request->getParam("input2")->value().length() > 0) {
      strncpy(myParameter2, request->getParam("input2")->value().c_str(), sizeof(myParameter2) - 1);
      myParameter2[sizeof(myParameter2) - 1] = '\0';
    }
    if (request->hasParam("input3") && request->getParam("input3")->value().length() > 0) {
      strncpy(myParameter3, request->getParam("input3")->value().c_str(), sizeof(myParameter3) - 1);
      myParameter3[sizeof(myParameter3) - 1] = '\0';
    }
    if (request->hasParam("input4") && request->getParam("input4")->value().length() > 0) {
      strncpy(myParameter4, request->getParam("input4")->value().c_str(), sizeof(myParameter4) - 1);
      myParameter4[sizeof(myParameter4) - 1] = '\0';
    }
    if (request->hasParam("input5") && request->getParam("input5")->value().length() > 0) {
      strncpy(myParameter5, request->getParam("input5")->value().c_str(), sizeof(myParameter5) - 1);
      myParameter5[sizeof(myParameter5) - 1] = '\0';
    }

    Serial.println("Settings Updated");
    request->send(200, "text/html", "Configuration updated.<br><a href=\"/\">Return</a>");
  });

  // Set the fallback for unhandled routes
  server.onNotFound(notFound);

  // Start the server ONLY ONCE
  server.begin();

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/ || defined(USBCON) /*STM32_stm32*/ \
  || defined(SERIALUSB_PID) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
  delay(4000);  // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
  Serial.println(F("Send IR signals at pin " STR(IR_SEND_PIN)));

  /*
     * The IR library setup. That's all!
     */
  IrSender.begin();  // Start with IR_SEND_PIN -which is defined in PinDefinitionsAndMore.h- as send pin and enable feedback LED at default feedback LED pin

  Serial.println();
  MyLG_Aircondition.setType(LG_IS_WALL_TYPE);
  MyLG_Aircondition.printMenu(&Serial);

  TimeIntervalPrevious = millis();
  u8g2.begin();  //initialise OLED Display
  OLEDUpdateIntervalPrevious = millis();

  am2320.begin(); // The sensor joins the same bus
}

void loop() {

  unsigned long currentMillis = millis();
  

  // --- NEW RECONNECTION LOGIC ---
  // If WiFi is down AND 10 seconds have passed, try to reconnect
  if (WiFi.status() != WL_CONNECTED && (currentMillis - previousWifiCheckMillis >= wifiCheckInterval)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();  // or WiFi.begin(ssid, password);
    previousWifiCheckMillis = currentMillis;
    Wifi_State = "reconnecting";
  }
  // ------------------------------

  // --- Add Periodic Time Synchronization Check ---
  if (WiFi.status() == WL_CONNECTED && (millis() - TimeIntervalPrevious >= TimeSyncInterval)) {
    // Re-run configuration and sync the time
    //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    configTzTime(TZ_INFO, ntpServer);
    printLocalTime();  // This is where the actual time sync is requested

    TimeIntervalPrevious = millis();
  }


  if (atoi(myParameter1) != 0 || (myParameter1[0] == '0' && myParameter1[1] == '\0')) {  // Check if it's non-zero or just "0"
    //If the value is non-zero then it means i have entered a new value
    AC_Turn_On_Time_Hour = atoi(myParameter1);
    AC_Turn_On_Time_Minute = atoi(myParameter2);
  }

  // Check temperature bounds
  if (atoi(myParameter3) >= 18 && atoi(myParameter3) <= 29) {
    TargetTemperature = atoi(myParameter3);
  }

  // Compare mode (case-insensitive for 'h')
  // strncasecmp(s1, s2, n) returns 0 if the first n characters are equal (case-insensitive)
  if (strncasecmp(myParameter4, "h", 1) == 0) {
    //we have set it to heat mode
    myModeParameter = AC_MODE_HEATING - '0';
    // process the heating commands so its only the bits it needs, i dont know what this does ¯\_(ツ)_/¯
    AC_Mode_State = "Heat";
  }
  // Compare mode (case-insensitive for 'c')
  else if (strncasecmp(myParameter4, "c", 1) == 0) {
    //we have set it to cool mode
    myModeParameter = AC_MODE_COOLING - '0';
    // process the heating commands so its only the bits it needs, i dont know what this does ¯\_(ツ)_/¯
    AC_Mode_State = "Cool";
  } else {
    //we have an error, do nothing
    AC_Mode_State = "not set";
  }

  // Check runSystem (1)
  if (atoi(myParameter5) == 1) {
    runSystem = HIGH;
  }

  // Check runSystem (0)
  if (atoi(myParameter5) == 0) {
    runSystem = LOW;
  }


  if (runSystem == HIGH) {
    RunSystemStr = "RUN";
  }
  if (runSystem == LOW) {
    RunSystemStr = "OFF";
  }

  if (millis() - OLEDUpdateIntervalPrevious >= OLEDUpdateInterval) {
    updateSensorReadings();// get current temperature and humidity reading
    getLocalTime(&timenow, 0);
    strftime(DisplayTimeWeekDay, 10, "%A", &timenow);
    strftime(DisplayTimeHour, 3, "%H", &timenow);
    strftime(DisplayTimeMinute, 3, "%M", &timenow);
    sprintf(displayStartTimeHour, "%02d", AC_Turn_On_Time_Hour);
    sprintf(displayStartTimeMinute, "%02d", AC_Turn_On_Time_Minute);

    u8g2.clearBuffer();
    u8g2.enableUTF8Print();
    u8g2.setFont(u8g2_font_smart_patrol_nbp_tf);  // choose a small font
    //draw element: x_pos, y_pos, variable
    u8g2.drawStr(20, 10, DisplayTimeWeekDay);  //write out the day of the week
    //Wifi_State and IP Address
    u8g2.setCursor(0, 110);
    u8g2.print(Wifi_State);
    u8g2.setCursor(0, 125);
    u8g2.print(WiFi.localIP());


    u8g2.drawStr(0, 50, "On Time");
    u8g2.setCursor(70, 50);
    u8g2.print(displayStartTimeHour);
    u8g2.drawStr(92, 50, ":");
    u8g2.setCursor(95, 50);
    u8g2.print(displayStartTimeMinute);

    u8g2.drawStr(0, 65, "Set point (C)");
    u8g2.setCursor(100, 65);
    u8g2.print(TargetTemperature);

    u8g2.drawStr(0, 80, "System");
    u8g2.setCursor(70, 80);
    u8g2.print(RunSystemStr);
    u8g2.drawStr(0, 95, "Mode");
    u8g2.setCursor(50, 95);
    u8g2.print(AC_Mode_State);

    u8g2.setFont(u8g2_font_profont22_tf);  // choose a bigger font for time
    u8g2.drawStr(0, 30, DisplayTimeHour);
    u8g2.drawStr(22, 30, ":");
    u8g2.drawStr(31, 30, DisplayTimeMinute);
    u8g2.drawStr(60, 30, "/");
    u8g2.setCursor(80, 30);
    u8g2.print(currentTemp, 1); // Display with 1 decimal place
    u8g2.sendBuffer();
    OLEDUpdateIntervalPrevious = millis();
  }



  /*
   * ========================================================================
   * TURN ON THE SYSTEM, WITH HIGH FAN SPEED TO WARM UP THE ROOM QUICKLY
   * ========================================================================
   */
  // Find the AC Turn-On logic around source 85
  if (runSystem == HIGH && AC_State_Previous == 0 && timenow.tm_hour == AC_Turn_On_Time_Hour && timenow.tm_min == AC_Turn_On_Time_Minute) {
    // Instead of sending commands immediately, start the sequence
    if (IRCommandStep == 0) {  // Only start if we are idle
      IRCommandStep = 1;
      previousIRMillis = millis();  // Initialize the timer
      AC_State_Previous = 1;        // Block re-triggering for this minute
    }
    SystemState = "HIGH";
  }
  //After a period of time we need ot clear AC_State_Previous so it can run again the next day. Propose to link this to a time.
  if (AC_State_Previous == 1 && timenow.tm_hour == AC_Turn_On_Time_Hour && timenow.tm_min == (AC_Turn_On_Time_Minute + 1)) {
    //After a delay, we reset AC_State_Previous
    AC_State_Previous = 0;  //reset previous state to zero.
  }

  /*
   * ========================================================================
   * TURN THE SYSTEM OFF
   * ========================================================================
   */
  if (runSystem == HIGH && AC_State_Previous == 0 && timenow.tm_hour == AC_Turn_Off_Time_Hour && timenow.tm_min == AC_Turn_Off_Time_Minute) {
    // send command to turn AC OFF
    MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_OFF, 0);
    AC_State_Previous = 1;  // this helps to stop us spamming commands for an entire minute during switch on
    SystemState = "OFF";
  }
  //After a period of time we need ot clear AC_State_Previous so it can run again the next day. Propose to link this to a time.
  if (AC_State_Previous == 1 && timenow.tm_hour == AC_Turn_Off_Time_Hour && timenow.tm_min == (AC_Turn_Off_Time_Minute + 1)) {
    AC_State_Previous = 0;  //reset previous state to zero.
  }

  // --- Non-Blocking IR Transmission Sequence ---
  if (IRCommandStep > 0 && (millis() - previousIRMillis >= IRInterval)) {

    // Check which command to send
    switch (IRCommandStep) {
      case 1:
        Serial.println("IR Step 1: Sending ON");
        MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_ON, 0);
        IRCommandStep = 2;  // Move to the next step
        break;

      case 2:
        Serial.println("IR Step 2: Sending MODE");
        MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_MODE, myModeParameter);  // set the AC mode
        IRCommandStep = 3;                                                            // Move to the next step
        break;

      case 3:
        Serial.println("IR Step 3: Sending TEMPERATURE");
        MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_TEMPERATURE, TargetTemperature);  // set the AC temperature
        IRCommandStep = 4;                                                                     // Move to the final step
        break;

      case 4:
        Serial.println("IR Sequence Complete.");
        // All commands sent. Reset the sequence.
        IRCommandStep = 0;
        break;
    }

    previousIRMillis = millis();  // Reset the timer for the next step
  }
  // --- End Non-Blocking IR Transmission Sequence ---
}


void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    TimeSyncSuccessBool = false;
    return;
  } else {
    TimeSyncSuccessBool = true;
  }
}

void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case ARDUINO_EVENT_WIFI_READY:
      Serial.println("WiFi ready");
      Wifi_State = "ready";
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("WiFi station started");
      Wifi_State = "started";
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("Connected to WiFi network");
      Wifi_State = "connected";
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("WiFi lost connection...");
      // *** CRITICAL FIX HERE *** // We REMOVED WiFi.begin() from here.
      // We let the main loop() handle the reconnection every 10 seconds.
      TimeSyncSuccessBool = false;
      Wifi_State = "disconnected";
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      // Re-initialize and get the time after a successful connection
      //configTime(gmtOffset_sec, daylightOffset_sec, ntpServer); //replacing with below
      configTzTime(TZ_INFO, ntpServer);
      printLocalTime();
      Wifi_State = "GOT IP";
      break;
    default:
      break;
  }
}

void updateSensorReadings() {
  // Only read every 5 seconds; AM2320 is slow and hates being rushed
  static unsigned long lastRead = 0;
  if (millis() - lastRead < 5000) return;
  lastRead = millis();

  // The library handles the I2C wake-up pulse automatically
  currentTemp = am2320.readTemperature();
  currentHum = am2320.readHumidity();

  if (isnan(currentTemp) || isnan(currentHum)) {
    Serial.println("AM2320 Error: Check I2C wiring (SDA/SCL)");
  }
}