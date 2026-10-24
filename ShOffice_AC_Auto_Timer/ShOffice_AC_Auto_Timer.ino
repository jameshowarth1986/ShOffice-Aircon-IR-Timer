// --- 1. Libraries ---
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <time.h>
#include <U8g2lib.h>
#include "PinDefinitionsAndMore.h"

#ifndef INFO
#define INFO(x)
#endif

#include <IRremote.hpp>

#include "ac_LG.hpp"
#include "Adafruit_MCP9808.h"


// --- 2. Hardware Objects ---
AsyncWebServer server(80);
Aircondition_LG MyLG_Aircondition;
U8G2_SH1107_SEEED_128X128_F_HW_I2C u8g2(U8G2_R0, SCL, SDA, U8X8_PIN_NONE);
Adafruit_MCP9808 mcp = Adafruit_MCP9808();

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
// Add this line to track the 7 days (0=Sun, 1=Mon, 2=Tue, 3=Wed, 4=Thu, 5=Fri, 6=Sat)
bool activeDays[7] = {true, true, true, true, true, true, true}; 
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
int AM2320UpdateIntervalPrevious = 0;
int AM2320UpdateInterval = 10000;

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

// Forward declarations
void updateSensorReadings();
void printLocalTime();
void WiFiEvent(WiFiEvent_t event);

// --- 8. WebPage Presentation converter ---
String processor(const String& var) {
  if (var == "HOUR") return String(myParameter1);
  if (var == "MIN") return String(myParameter2);
  if (var == "TEMP") return String(myParameter3);
  if (var == "MODE") return String(myParameter4);
  if (var == "RUN") return String(myParameter5);
  if (var == "CURRENT_TEMP") return String(currentTemp, 1);

  // Mode radio state
  if (var == "MODE_H_CHECKED") return (strncasecmp(myParameter4, "h", 1) == 0) ? "checked" : "";
  if (var == "MODE_C_CHECKED") return (strncasecmp(myParameter4, "c", 1) == 0) ? "checked" : "";

  // Run radio state
  if (var == "RUN_1_CHECKED") return (atoi(myParameter5) == 1) ? "checked" : "";
  if (var == "RUN_0_CHECKED") return (atoi(myParameter5) == 0) ? "checked" : "";

  // Day selection placeholders
  if (var == "CHK_SUN") return activeDays[0] ? "checked" : "";
  if (var == "CHK_MON") return activeDays[1] ? "checked" : "";
  if (var == "CHK_TUE") return activeDays[2] ? "checked" : "";
  if (var == "CHK_WED") return activeDays[3] ? "checked" : "";
  if (var == "CHK_THU") return activeDays[4] ? "checked" : "";
  if (var == "CHK_FRI") return activeDays[5] ? "checked" : "";
  if (var == "CHK_SAT") return activeDays[6] ? "checked" : "";

  return String();
}

// Light Mode UI Template for ESPAsyncWebServer
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>AC Controller</title>
  <link rel="preconnect" href="https://fonts.googleapis.com">
  <link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
  <link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@400;500;600;700;800&display=swap" rel="stylesheet">
  <style>
    :root {
      --bg: #F4F6F9;
      --card-bg: #FFFFFF;
      --text-main: #1E293B;
      --text-muted: #64748B;
      --border: #E2E8F0;
      --accent-cool: #0284C7;
      --accent-cool-light: #E0F2FE;
      --accent-heat: #EA580C;
      --accent-heat-light: #FFEDD5;
      --accent-green: #16A34A;
      --accent-green-light: #DCFCE7;
      --danger: #EF4444;
      --shadow-sm: 0 2px 4px rgba(0,0,0,0.02), 0 1px 2px rgba(0,0,0,0.03);
      --shadow-md: 0 10px 25px -5px rgba(0,0,0,0.05), 0 8px 10px -6px rgba(0,0,0,0.01);
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: 'Plus Jakarta Sans', sans-serif; }
    body { background-color: var(--bg); color: var(--text-main); min-height: 100vh; display: flex; justify-content: center; align-items: flex-start; padding: 20px 12px; }
    .container { width: 100%%; max-width: 440px; margin: 0 auto; }
    .card { background: var(--card-bg); border-radius: 24px; padding: 24px; box-shadow: var(--shadow-md); border: 1px solid var(--border); margin-bottom: 16px; }
    .header { text-align: center; margin-bottom: 24px; }
    .header h1 { font-size: 1.5rem; font-weight: 800; color: var(--text-main); }
    .temp-card { background: linear-gradient(135deg, #F8FAFC 0%%, #EFF6FF 100%%); border: 1px solid #DBEAFE; border-radius: 20px; padding: 20px; text-align: center; position: relative; margin-bottom: 24px; }
    .temp-label { font-size: 0.85rem; font-weight: 700; color: var(--accent-cool); text-transform: uppercase; letter-spacing: 0.5px; }
    .temp-val { font-size: 3.2rem; font-weight: 800; color: var(--text-main); margin: 6px 0; display: flex; align-items: center; justify-content: center; }
    .form-group { margin-bottom: 20px; }
    .form-label { display: block; font-size: 0.85rem; font-weight: 700; color: var(--text-muted); margin-bottom: 8px; text-transform: uppercase; letter-spacing: 0.5px; }
    .segmented-control { display: flex; background: #F1F5F9; padding: 4px; border-radius: 14px; gap: 4px; }
    .segmented-control input[type="radio"] { display: none; }
    .segmented-control label { flex: 1; text-align: center; padding: 10px; font-size: 0.9rem; font-weight: 700; border-radius: 10px; cursor: pointer; color: var(--text-muted); transition: all 0.2s ease; }
    .segmented-control input[type="radio"]:checked + label { background: #FFFFFF; color: var(--text-main); box-shadow: var(--shadow-sm); }
    #mode-h:checked + label { color: var(--accent-heat); }
    #mode-c:checked + label { color: var(--accent-cool); }
    .input-box { width: 100%%; background: #F8FAFC; border: 1.5px solid var(--border); border-radius: 12px; padding: 12px 16px; font-size: 1rem; font-weight: 600; color: var(--text-main); outline: none; transition: border-color 0.2s; }
    .input-box:focus { border-color: var(--accent-cool); background: #FFFFFF; }
    .time-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
    .days-grid { display: grid; grid-template-columns: repeat(7, 1fr); gap: 4px; }
    .day-pill input { display: none; }
    .day-pill label { display: flex; align-items: center; justify-content: center; height: 38px; border-radius: 10px; background: #F1F5F9; font-size: 0.8rem; font-weight: 700; color: var(--text-muted); cursor: pointer; transition: all 0.2s ease; }
    .day-pill input:checked + label { background: var(--text-main); color: #FFFFFF; }
    .btn { width: 100%%; padding: 16px; border: none; border-radius: 14px; font-size: 1rem; font-weight: 700; cursor: pointer; transition: transform 0.1s ease, box-shadow 0.2s ease; display: block; text-align: center; text-decoration: none; box-shadow: var(--shadow-sm); }
    .btn:active { transform: scale(0.98); }
    .btn-primary { background: var(--text-main); color: #FFFFFF; }
    .btn-success { background: var(--accent-green); color: #FFFFFF; margin-bottom: 12px; }
    .btn-danger { background: #FEE2E2; color: var(--danger); }
    .footer-link { text-align: center; margin-top: 16px; }
    .footer-link a { color: var(--text-muted); font-size: 0.9rem; font-weight: 600; text-decoration: none; }
  </style>
</head>
<body>
  <div class="container">
    <div class="card">
      <div class="header">
        <h1>AC Configuration</h1>
      </div>

      <div class="temp-card">
        <div class="temp-label">Current Office Temp</div>
        <div class="temp-val">%CURRENT_TEMP%&deg;C</div>
      </div>

      <form action="/get" method="GET">
        <div class="form-group">
          <label class="form-label">System Mode</label>
          <div class="segmented-control">
            <input type="radio" id="mode-h" name="input4" value="h" %MODE_H_CHECKED%>
            <label for="mode-h">Heat (h)</label>
            <input type="radio" id="mode-c" name="input4" value="c" %MODE_C_CHECKED%>
            <label for="mode-c">Cool (c)</label>
          </div>
        </div>

        <div class="form-group">
          <label class="form-label">System Power</label>
          <div class="segmented-control">
            <input type="radio" id="run-1" name="input5" value="1" %RUN_1_CHECKED%>
            <label for="run-1">On (1)</label>
            <input type="radio" id="run-0" name="input5" value="0" %RUN_0_CHECKED%>
            <label for="run-0">Off (0)</label>
          </div>
        </div>

        <div class="form-group">
          <label class="form-label">Target Setpoint (&deg;C)</label>
          <input type="number" class="input-box" name="input3" value="%TEMP%" placeholder="%TEMP%" min="18" max="29">
        </div>

        <div class="form-group">
          <label class="form-label">Schedule Start Time</label>
          <div class="time-grid">
            <div>
              <input type="number" class="input-box" name="input1" value="%HOUR%" placeholder="Hour (%HOUR%)" min="0" max="23">
            </div>
            <div>
              <input type="number" class="input-box" name="input2" value="%MIN%" placeholder="Min (%MIN%)" min="0" max="59">
            </div>
          </div>
        </div>

        <div class="form-group">
          <label class="form-label">Active Days</label>
          <div class="days-grid">
            <div class="day-pill">
              <input type="checkbox" id="d1" name="d1" value="1" %CHK_MON%>
              <label for="d1">M</label>
            </div>
            <div class="day-pill">
              <input type="checkbox" id="d2" name="d2" value="1" %CHK_TUE%>
              <label for="d2">T</label>
            </div>
            <div class="day-pill">
              <input type="checkbox" id="d3" name="d3" value="1" %CHK_WED%>
              <label for="d3">W</label>
            </div>
            <div class="day-pill">
              <input type="checkbox" id="d4" name="d4" value="1" %CHK_THU%>
              <label for="d4">T</label>
            </div>
            <div class="day-pill">
              <input type="checkbox" id="d5" name="d5" value="1" %CHK_FRI%>
              <label for="d5">F</label>
            </div>
            <div class="day-pill">
              <input type="checkbox" id="d6" name="d6" value="1" %CHK_SAT%>
              <label for="d6">S</label>
            </div>
            <div class="day-pill">
              <input type="checkbox" id="d0" name="d0" value="1" %CHK_SUN%>
              <label for="d0">S</label>
            </div>
          </div>
        </div>

        <button type="submit" class="btn btn-primary">Update Settings</button>
      </form>
    </div>

    <div class="card">
      <a href="/trigger" class="btn btn-success">SEND CONFIG SEQUENCE</a>
      <a href="/off" class="btn btn-danger">TURN AC OFF NOW</a>
      <div class="footer-link">
        <a href="/">Refresh Status</a>
      </div>
    </div>
  </div>
</body>
</html>
)rawliteral";

const char success_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Settings Updated</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@500;700&display=swap" rel="stylesheet">
  <style>
    body { font-family: 'Plus Jakarta Sans', sans-serif; background: #F4F6F9; text-align: center; margin-top: 50px; }
    .card { display: inline-block; background: #ffffff; padding: 30px; border-radius: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05); }
    h2 { color: #16A34A; margin-bottom: 10px; }
    .btn { background-color: #1E293B; color: white; border: none; padding: 12px 24px; border-radius: 10px; cursor: pointer; text-decoration: none; font-weight: bold; display: inline-block; margin-top: 20px; }
  </style>
</head><body>
  <div class="card">
    <h2>✔ Settings Updated</h2>
    <p>Your AC configuration has been saved successfully.</p>
    <a href="/" class="btn">Return to Dashboard</a>
  </div>
</body></html>)rawliteral";

const char sequence_initiated[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Sequence Initiated</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@500;700&display=swap" rel="stylesheet">
  <style>
    body { font-family: 'Plus Jakarta Sans', sans-serif; background: #F4F6F9; text-align: center; margin-top: 50px; }
    .card { display: inline-block; background: #ffffff; padding: 30px; border-radius: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05); }
    h2 { color: #16A34A; margin-bottom: 10px; }
    .btn { background-color: #1E293B; color: white; border: none; padding: 12px 24px; border-radius: 10px; cursor: pointer; text-decoration: none; font-weight: bold; display: inline-block; margin-top: 20px; }
  </style>
</head><body>
  <div class="card">
    <h2>✔ IR Command Initiated</h2>
    <p>The IR command sequence has been initiated.</p>
    <a href="/" class="btn">Return to Dashboard</a>
  </div>
</body></html>)rawliteral";

const char power_off_cmd_sent[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Power Off Sent</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <link href="https://fonts.googleapis.com/css2?family=Plus+Jakarta+Sans:wght@500;700&display=swap" rel="stylesheet">
  <style>
    body { font-family: 'Plus Jakarta Sans', sans-serif; background: #F4F6F9; text-align: center; margin-top: 50px; }
    .card { display: inline-block; background: #ffffff; padding: 30px; border-radius: 20px; box-shadow: 0 4px 12px rgba(0,0,0,0.05); }
    h2 { color: #EF4444; margin-bottom: 10px; }
    .btn { background-color: #1E293B; color: white; border: none; padding: 12px 24px; border-radius: 10px; cursor: pointer; text-decoration: none; font-weight: bold; display: inline-block; margin-top: 20px; }
  </style>
</head><body>
  <div class="card">
    <h2>✔ Power Off Command Initiated</h2>
    <p>The Air Conditioning unit will now be switched off.</p>
    <a href="/" class="btn">Return to Dashboard</a>
  </div>
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
  WiFi.setTxPower(WIFI_POWER_8_5dBm);  
  WiFi.begin(ssid, password);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  server.on("/trigger", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (IRCommandStep == 0) {  // Only start if not already sending
      IRCommandStep = 1;
      previousIRMillis = millis();  // Initialize the non-blocking timer
      Serial.println("Manual IR Triggered via Web");
      request->send(200, "text/html", sequence_initiated);
    } else {
      request->send(200, "text/html", "Sequence already in progress.<br><a href=\"/\">Return</a>");
    }
  });

  // Button for sending off command
  server.on("/off", HTTP_GET, [](AsyncWebServerRequest* request) {
    MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_OFF, 0); 
    Serial.println("Manual IR OFF Triggered via Web");
    request->send_P(200, "text/html", power_off_cmd_sent); 
  });

  // Handle incoming settings form submit
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest* request) {
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
    
    // Reset days
    for(int i = 0; i < 7; i++) {
      activeDays[i] = false;
    }
    
    // Read days
    if (request->hasParam("d0")) activeDays[0] = true;
    if (request->hasParam("d1")) activeDays[1] = true;
    if (request->hasParam("d2")) activeDays[2] = true;
    if (request->hasParam("d3")) activeDays[3] = true;
    if (request->hasParam("d4")) activeDays[4] = true;
    if (request->hasParam("d5")) activeDays[5] = true;
    if (request->hasParam("d6")) activeDays[6] = true;

    Serial.println("Settings Updated");
    request->send_P(200, "text/html", success_html);
  });

  server.onNotFound(notFound);
  server.begin();

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) || defined(USBCON) \
  || defined(SERIALUSB_PID) || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
  delay(4000); 
#endif

  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
  Serial.println(F("Send IR signals at pin " STR(IR_SEND_PIN)));

  IrSender.begin();  

  Serial.println();
  MyLG_Aircondition.setType(LG_IS_WALL_TYPE);
  MyLG_Aircondition.printMenu(&Serial);

  TimeIntervalPrevious = millis();
  u8g2.begin();  
  OLEDUpdateIntervalPrevious = millis();
  AM2320UpdateIntervalPrevious = millis();

  if (!mcp.begin(0x18)) { 
    Serial.println("Could not find MCP9808! Check wiring.");
  } 
  else {
    mcp.setResolution(3); 
    Serial.println("MCP9808 found!");
  }

  Wire.setClock(100000);
}

void loop() {
  unsigned long currentMillis = millis();

  if (WiFi.status() != WL_CONNECTED && (currentMillis - previousWifiCheckMillis >= wifiCheckInterval)) {
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();  
    previousWifiCheckMillis = currentMillis;
    Wifi_State = "reconnecting";
  }

  if (WiFi.status() == WL_CONNECTED && (millis() - TimeIntervalPrevious >= TimeSyncInterval)) {
    configTzTime(TZ_INFO, ntpServer);
    printLocalTime();  
    TimeIntervalPrevious = millis();
  }

  if (atoi(myParameter1) != 0 || (myParameter1[0] == '0' && myParameter1[1] == '\0')) {  
    AC_Turn_On_Time_Hour = atoi(myParameter1);
    AC_Turn_On_Time_Minute = atoi(myParameter2);
  }

  if (atoi(myParameter3) >= 18 && atoi(myParameter3) <= 29) {
    TargetTemperature = atoi(myParameter3);
  }

  if (strncasecmp(myParameter4, "h", 1) == 0) {
    myModeParameter = AC_MODE_HEATING - '0';
    AC_Mode_State = "Heat";
  }
  else if (strncasecmp(myParameter4, "c", 1) == 0) {
    myModeParameter = AC_MODE_COOLING - '0';
    AC_Mode_State = "Cool";
  } else {
    AC_Mode_State = "not set";
  }

  if (atoi(myParameter5) == 1) runSystem = HIGH;
  if (atoi(myParameter5) == 0) runSystem = LOW;

  RunSystemStr = (runSystem == HIGH) ? "RUN" : "OFF";

  if (millis() - AM2320UpdateIntervalPrevious >= AM2320UpdateInterval){
    updateSensorReadings();
    AM2320UpdateIntervalPrevious = millis(); 
  }

  if (millis() - OLEDUpdateIntervalPrevious >= OLEDUpdateInterval) {
    getLocalTime(&timenow, 0);
    strftime(DisplayTimeWeekDay, 10, "%A", &timenow);
    strftime(DisplayTimeHour, 3, "%H", &timenow);
    strftime(DisplayTimeMinute, 3, "%M", &timenow);
    sprintf(displayStartTimeHour, "%02d", AC_Turn_On_Time_Hour);
    sprintf(displayStartTimeMinute, "%02d", AC_Turn_On_Time_Minute);

    u8g2.clearBuffer();
    u8g2.enableUTF8Print();
    u8g2.setFont(u8g2_font_smart_patrol_nbp_tf);  
    u8g2.drawStr(20, 10, DisplayTimeWeekDay);  
    
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

    u8g2.setFont(u8g2_font_profont22_tf);  
    u8g2.drawStr(0, 30, DisplayTimeHour);
    u8g2.drawStr(22, 30, ":");
    u8g2.drawStr(31, 30, DisplayTimeMinute);
    u8g2.drawStr(60, 30, "/");
    u8g2.setCursor(80, 30);
    u8g2.print(currentTemp, 1); 
    u8g2.sendBuffer();
    OLEDUpdateIntervalPrevious = millis();
  }

  // --- AC TURN ON LOGIC ---
  if (runSystem == HIGH && AC_State_Previous == 0 && timenow.tm_hour == AC_Turn_On_Time_Hour && timenow.tm_min == AC_Turn_On_Time_Minute && activeDays[timenow.tm_wday] == true) {
    if (IRCommandStep == 0) {  
      IRCommandStep = 1;
      previousIRMillis = millis();  
      AC_State_Previous = 1;        
    }
    SystemState = "HIGH";
  }

  if (AC_State_Previous == 1 && timenow.tm_hour == AC_Turn_On_Time_Hour && timenow.tm_min == (AC_Turn_On_Time_Minute + 1)) {
    AC_State_Previous = 0;  
  }

  // --- AC TURN OFF LOGIC ---
  if (runSystem == HIGH && AC_State_Previous == 0 && timenow.tm_hour == AC_Turn_Off_Time_Hour && timenow.tm_min == AC_Turn_Off_Time_Minute && activeDays[timenow.tm_wday] == true) {
    MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_OFF, 0);
    AC_State_Previous = 1;  
    SystemState = "OFF";
  }

  if (AC_State_Previous == 1 && timenow.tm_hour == AC_Turn_Off_Time_Hour && timenow.tm_min == (AC_Turn_Off_Time_Minute + 1)) {
    AC_State_Previous = 0;  
  }

  // --- Non-Blocking IR Transmission Sequence ---
  if (IRCommandStep > 0 && (millis() - previousIRMillis >= IRInterval)) {
    switch (IRCommandStep) {
      case 1:
        Serial.println("IR Step 1: Sending ON");
        MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_ON, 0);
        IRCommandStep = 2;  
        break;

      case 2:
        Serial.println("IR Step 2: Sending MODE");
        MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_MODE, myModeParameter);  
        IRCommandStep = 3;                                                            
        break;

      case 3:
        Serial.println("IR Step 3: Sending TEMPERATURE");
        MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_TEMPERATURE, TargetTemperature);  
        IRCommandStep = 4;                                                                     
        break;

      case 4:
        Serial.println("IR Sequence Complete.");
        IRCommandStep = 0;
        break;
    }

    previousIRMillis = millis();  
  }
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
      TimeSyncSuccessBool = false;
      Wifi_State = "disconnected";
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      configTzTime(TZ_INFO, ntpServer);
      printLocalTime();
      Wifi_State = "GOT IP";
      break;
    default:
      break;
  }
}

void updateSensorReadings() {
  static unsigned long lastRead = 0;
  if (millis() - lastRead < 10000) return; 
  lastRead = millis();

  mcp.wake(); 
  float t = mcp.readTempC();
  mcp.shutdown_wake(1); 

  if (isnan(t)) {
    Serial.println("MCP9808 Read Failed");
    currentTemp = 99; 
    return;
  }

  currentTemp = t;
}