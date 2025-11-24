/*
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-date-time-ntp-client-server-arduino/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h> //using version 3.6.0 works. careful when updating
#include <time.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_AM2320.h>

AsyncWebServer server(80);

const char* PARAM_INPUT_1 = "input1";
const char* PARAM_INPUT_2 = "input2";
const char* PARAM_INPUT_3 = "input3";
const char* PARAM_INPUT_4 = "input4";
const char* PARAM_INPUT_5 = "input5";

String myParameter1 = "new";
String myParameter2 = "new";
String myParameter3 = "new";
String myParameter4 = "new";
String myParameter5 = "new";

char displayStartTimeHour [2]= {'h','h'};
char displayStartTimeMinute [2] = {'m','m'};

bool FirstLoopComplete = 0;


int myInteger1 = 0;
int myInteger2 = 0;
int myInteger3 = 0;

//==============================================================
// INSERT ALL OF THE AIRCON IR CONTROL STUFF
//==============================================================
  /*
  * LG2 has different header timing and a shorter bit time
  * Known LG remote controls, which uses LG2 protocol are:
  * AKB75215403
  * AKB74955603
  * AKB73757604:
  */
  //#define USE_LG2_PROTOCOL // Try it if you do not have success with the default LG protocol
  #define NUMBER_OF_COMMANDS_BETWEEN_PRINT_OF_MENU 5

  #define DISABLE_CODE_FOR_RECEIVER // Disables restarting receiver after each send. Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not used.

  #define INFO // Deactivate this to save program memory and suppress info output from the LG-AC driver.
  //#define DEBUG // Activate this for more output from the LG-AC driver.

  #include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
  #include <IRremote.hpp>

  #if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
  #include "ATtinySerialOut.hpp" // Available as Arduino library "ATtinySerialOut"
  #endif

  #include "ac_LG.hpp"

  #define SIZE_OF_RECEIVE_BUFFER 10
  char sRequestString[SIZE_OF_RECEIVE_BUFFER];

  Aircondition_LG MyLG_Aircondition;
//==============================================================
// END OF ALL OF THE AIRCON IR CONTROL STUFF
//==============================================================


Adafruit_AM2320 am2320 = Adafruit_AM2320();

// HTML web page to handle 3 input fields (input1, input2, input3)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP Input Form</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head><body>
  <form action="/get">
    Hour Start: <input type="text" name="input1">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    Minute Start: <input type="text" name="input2">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    Target Temperature (C): <input type="text" name="input3">
    <input type="submit" value="Submit">
  </form><br>
  <form action="/get">
    AC Mode (h/c): <input type="text" name="input4">
    <input type="submit" value="Submit">
  </form>
  <form action="/get">
    System Run (1/0): <input type="text" name="input5">
    <input type="submit" value="Submit">
  </form>
</body></html>)rawliteral";

void notFound(AsyncWebServerRequest* request) {
  request->send(404, "text/plain", "Not found");
}


const char* ssid     = "ShOffice_2_4GHz";
const char* password = "workworkwork";

//const char* ssid = "VM2681428";
//const char* password = "Zt5hgrrw8Ktc";

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 0;
const int   daylightOffset_sec = 0; //For use in British summer time set to 3600, for winter set to 0(BST)
//#define runPin 23 //make sure there is a pull down resistor on the button, because it is a normally open type and if left floating the microcontroller gets confused. 
#define GreenGoPin 5 // for green button
#define RedStopPin 6
bool runSystem = 0;
String SystemState = "OFF";
String AC_Mode_State = "null";
String RunSystemStr = "INITIALISED";
String CurrentState = "INITIALISED";
String PreviousState = "INITIALISED";
String Wifi_State = "unknown";

unsigned long previousIRMillis = 0;
const long IRInterval = 2000; // 2 seconds delay between IR commands
int IRCommandStep = 0; // 0=Idle, 1=Send ON, 2=Send MODE, 3=Send TEMP

int myCommand = 0;
int myParameter = 0;
int myModeParameter = 0;
int myTemperatureParameter = 0;
int myFanSpeedParameter = 0;
int TargetTemperature = 21;

//set up time variables for turning system on and off
int AC_Turn_On_Time_Hour = 05;
int AC_Turn_On_Time_Minute = 00;

const int AC_Mellow_Time_Hour = 07;
const int AC_Turn_Mellow_Minute = 00;

const int AC_Turn_Off_Time_Hour = 21;
const int AC_Turn_Off_Time_Minute = 00;

//set up timers so we only run some functions when we need to
//Intervals for syncing time with Wifi
int TimeSyncInterval = (1000*60*60*6); //synchronise time every 6 hours
int TimeIntervalPrevious = 0;

//Intervals for updating the OLEd
int OLEDUpdateInterval = 200; //200ms updates at 5Hz
int OLEDUpdateIntervalPrevious = 0;

//Intervals for getting temperature
int TemperatureUpdateInterval = 15000;// about 15 seconds
int TemperatureUpdateIntervalPrevious = 0;

//define a tracker for knowing when the AC has been off or on, like a current state and last state
//Treat 0 as off and 1 as on
bool AC_State_Previous = 0;
bool AC_State_Current = 0;

int HumidityNow = 0;
int TemperatureNow = 0;

//const int   daylightOffset_sec = 0; //For use in winter 

const int Christmas_Day = 358;
int       Days_Until_Christmas = 0;

//U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE); //no name OLED
//U8G2_SH1107_SEEED_128X128_1_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE); //Grove SEEED OLED Picture Loop Mode
U8G2_SH1107_SEEED_128X128_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE); //Grove SEEED OLED Full Page buffer mode - FAST

//Time variables
struct tm timenow;
char DisplayTimeWeekDay[10];
char DisplayTimeHour[3];
char DisplayTimeMinute[3];
char DisplayMidnightMinutes[5];
char DisplayTemperature[3];
char DisplayHumidity[3];

int timeSinceMidnightMinutes = 0;
bool TimeSyncSuccessBool = false;

void setup(){
  Serial.begin(115200);

  // 1. REGISTER THE EVENT HANDLER
  WiFi.onEvent(WiFiEvent); // This tells the ESP32 to run WiFiEvent() on network changes

  // 2. INITIAL CONNECTION
  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  // This starts the connection process, but does NOT wait.
  // The ARDUINO_EVENT_WIFI_STA_GOT_IP event will handle the rest.
  WiFi.begin(ssid, password);

  // 3. Keep configTime and server.begin() *after* WiFi.begin() but you don't need to wait for the IP here.
  // We recommend moving configTime and printLocalTime into the GOT_IP event handler for best practice.

  // 3. ASYNC WEB SERVER SETUP (Define ALL routes and start ONCE)
  
  // Route for the homepage ("/") 
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html);
  });

  // Route for the form submission ("/get")
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest* request) {
    String inputMessage;
    String inputParam;
    // GET input1 value on <ESP_IP>/get?input1=<inputMessage> 
    if (request->hasParam(PARAM_INPUT_1)) {
      inputMessage = request->getParam(PARAM_INPUT_1)->value();
      inputParam = PARAM_INPUT_1; 
      myParameter1 = inputMessage;
    }
    // GET input2 value on <ESP_IP>/get?input2=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_2)) {
      inputMessage = request->getParam(PARAM_INPUT_2)->value();
      inputParam = PARAM_INPUT_2; 
      myParameter2 = inputMessage;
    }
    // GET input3 value on <ESP_IP>/get?input3=<inputMessage> 
    else if (request->hasParam(PARAM_INPUT_3)) {
      inputMessage = request->getParam(PARAM_INPUT_3)->value();
      inputParam = PARAM_INPUT_3; 
      myParameter3 = inputMessage;
    } 
    // GET input4 value on <ESP_IP>/get?input4=<inputMessage>
    else if (request->hasParam(PARAM_INPUT_4)) {
      inputMessage = request->getParam(PARAM_INPUT_4)->value();
      inputParam = PARAM_INPUT_4;
      myParameter4 = inputMessage;
    }
    else if (request->hasParam(PARAM_INPUT_5)) {
      inputMessage = request->getParam(PARAM_INPUT_5)->value();
      inputParam = PARAM_INPUT_5; 
      myParameter5 = inputMessage;
    }
    else {
      inputMessage = "No message sent";
      inputParam = "none"; 
    }
    Serial.println(inputMessage); 
    //request->send(200, "text/html", "HTTP GET request sent to your ESP on input field (" + inputParam + ") with value: " + inputMessage + "<br><a href=\"/\">Return to Home Page</a>"); 
    request->send(200, "text/html", "Configuration updated successfully.<br><a href=\"/\">Return to Home Page</a>");
  });
  
  // Set the fallback for unhandled routes
  server.onNotFound(notFound);
  
  // Start the server ONLY ONCE
  server.begin(); 

  #if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
  delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
  #endif
  // Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
    Serial.println(F("Send IR signals at pin " STR(IR_SEND_PIN)));

    /*
     * The IR library setup. That's all!
     */
    IrSender.begin(); // Start with IR_SEND_PIN -which is defined in PinDefinitionsAndMore.h- as send pin and enable feedback LED at default feedback LED pin

    Serial.println();
    MyLG_Aircondition.setType(LG_IS_WALL_TYPE);
    MyLG_Aircondition.printMenu(&Serial);

  TimeIntervalPrevious = millis();
  u8g2.begin(); //initialise OLED Display 
  am2320.begin();
  //pinMode(runPin, INPUT);
  pinMode(GreenGoPin, INPUT);
  pinMode(RedStopPin, INPUT);
  TemperatureUpdateIntervalPrevious = millis();
  OLEDUpdateIntervalPrevious = millis();
}

void loop(){

  // --- Add Periodic Time Synchronization Check ---
    if (WiFi.status() == WL_CONNECTED && (millis() - TimeIntervalPrevious >= TimeSyncInterval)) {
      // Re-run configuration and sync the time
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      printLocalTime(); // This is where the actual time sync is requested

      TimeIntervalPrevious = millis();
    }

    
    if (WiFi.status() == WL_CONNECTED && !TimeSyncSuccessBool) {
      // Re-run configuration and sync the time
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      printLocalTime(); // This is where the actual time sync is requested

      TimeIntervalPrevious = millis();
    }


  if(myParameter1.toInt() != 0){
    //If the value is non-zero then it means i have entered a new value
    AC_Turn_On_Time_Hour = myParameter1.toInt();
    AC_Turn_On_Time_Minute = myParameter2.toInt();
  }

  if(myParameter3.toInt()>=18 && myParameter3.toInt()<=29){
    TargetTemperature = myParameter3.toInt();
  }

  if(myParameter4.equalsIgnoreCase("h")){
    //we have set it to heat mode
    myModeParameter = AC_MODE_HEATING - '0'; // process the heating commands so its only the bits it needs, i dont know what this does ¯\_(ツ)_/¯
    AC_Mode_State = "Heat";
  }
  else if(myParameter4.equalsIgnoreCase("c")){
    //we have set it to cool mode
    myModeParameter = AC_MODE_COOLING - '0'; // process the heating commands so its only the bits it needs, i dont know what this does ¯\_(ツ)_/¯
    AC_Mode_State = "Cool";
  }
  else{
    //we have an error, do nothing
    AC_Mode_State = "Error";
  }

  if(myParameter5.toInt() == 1){
    runSystem = HIGH;
  }

  if(myParameter5.toInt() == 0){
    runSystem = LOW;
  }
  
  if(AC_State_Previous == 1){
    PreviousState = "RUNNING";
  }
  else{
    PreviousState = "OFF";
  }

  if(runSystem == HIGH){
    RunSystemStr = "RUN";
  }
  if(runSystem == LOW){
    RunSystemStr = "OFF";
  }

  if(millis() - OLEDUpdateIntervalPrevious >= OLEDUpdateInterval){

    getLocalTime(&timenow, 0);
    strftime(DisplayTimeWeekDay,10, "%A", &timenow);
    strftime(DisplayTimeHour,3, "%H", &timenow);
    strftime(DisplayTimeMinute,3, "%M", &timenow);
    sprintf(displayStartTimeHour, "%02d", AC_Turn_On_Time_Hour);  
    sprintf(displayStartTimeMinute, "%02d", AC_Turn_On_Time_Minute);    

    u8g2.clearBuffer();
    u8g2.enableUTF8Print();
    u8g2.setFont(u8g2_font_smart_patrol_nbp_tf);	// choose a small font
    //draw element: x_pos, y_pos, variable
    u8g2.drawStr(20,10, DisplayTimeWeekDay);//write out the day of the week
    //Wifi_State and IP Address
    u8g2.setCursor(0, 110);
    u8g2.print(Wifi_State);
    u8g2.setCursor(0, 125);
    u8g2.print(WiFi.localIP());

    
    u8g2.drawStr(0,50, "On Time");
    u8g2.setCursor(70, 50);
    u8g2.print(displayStartTimeHour);
    u8g2.drawStr(92,50, ":");
    u8g2.setCursor(95, 50);
    u8g2.print(displayStartTimeMinute);

    u8g2.drawStr(0,65, "Set point (C)");
    u8g2.setCursor(100, 65);
    u8g2.print(TargetTemperature);
    
    u8g2.drawStr(0,80, "System");
    u8g2.setCursor(70, 80);
    u8g2.print(RunSystemStr);
    u8g2.drawStr(0,95, "Mode");
    u8g2.setCursor(70, 95);
    u8g2.print(AC_Mode_State);
    
    //u8g2.drawStr(0,110, "Previous:");
    //u8g2.setCursor(80, 110);
    //u8g2.print(PreviousState);

    u8g2.setFont(u8g2_font_profont22_tf);	// choose a bigger font for time
    u8g2.drawStr(37,30, DisplayTimeHour);
    u8g2.drawStr(59,30, ":");
    u8g2.drawStr(68,30, DisplayTimeMinute); 
    u8g2.sendBuffer();
    OLEDUpdateIntervalPrevious = millis();
  }
  
    
  
  /*
   * ========================================================================
   * TURN ON THE SYSTEM, WITH HIGH FAN SPEED TO WARM UP THE ROOM QUICKLY
   * ========================================================================
   */
    // Find the AC Turn-On logic around source 85
    if(runSystem == HIGH && AC_State_Previous == 0 && timenow.tm_hour == AC_Turn_On_Time_Hour && timenow.tm_min == AC_Turn_On_Time_Minute){
        // Instead of sending commands immediately, start the sequence
        if (IRCommandStep == 0) { // Only start if we are idle
            IRCommandStep = 1; 
            previousIRMillis = millis(); // Initialize the timer
            AC_State_Previous = 1; // Block re-triggering for this minute
        }
        SystemState = "HIGH";
    }
    //After a period of time we need ot clear AC_State_Previous so it can run again the next day. Propose to link this to a time.
    if(AC_State_Previous == 1 && timenow.tm_hour == AC_Turn_On_Time_Hour && timenow.tm_min == (AC_Turn_On_Time_Minute+1)){
      //After a delay, we reset AC_State_Previous
      AC_State_Previous = 0; //reset previous state to zero.
    }

  /*
   * ========================================================================
   * TURN THE SYSTEM OFF
   * ========================================================================
   */
    if(runSystem == HIGH && AC_State_Previous == 0 && timenow.tm_hour == AC_Turn_Off_Time_Hour && timenow.tm_min == AC_Turn_Off_Time_Minute){
      // send command to turn AC OFF
      MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_OFF, 0);
      AC_State_Previous = 1; // this helps to stop us spamming commands for an entire minute during switch on
      SystemState ="OFF";
    }
    //After a period of time we need ot clear AC_State_Previous so it can run again the next day. Propose to link this to a time.
    if(AC_State_Previous == 1 && timenow.tm_hour == AC_Turn_Off_Time_Hour && timenow.tm_min == (AC_Turn_Off_Time_Minute+1)){
      AC_State_Previous = 0; //reset previous state to zero.
    }

    // --- Non-Blocking IR Transmission Sequence ---
    if (IRCommandStep > 0 && (millis() - previousIRMillis >= IRInterval)) {
        
        // Check which command to send
        switch (IRCommandStep) {
            case 1:
                Serial.println("IR Step 1: Sending ON");
                MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_ON, 0); 
                IRCommandStep = 2; // Move to the next step
                break;
                
            case 2:
                Serial.println("IR Step 2: Sending MODE");
                MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_MODE, myModeParameter); // set the AC mode
                IRCommandStep = 3; // Move to the next step
                break;
                
            case 3:
                Serial.println("IR Step 3: Sending TEMPERATURE");
                MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_TEMPERATURE, TargetTemperature); // set the AC temperature
                IRCommandStep = 4; // Move to the final step
                break;
                
            case 4:
                Serial.println("IR Sequence Complete.");
                // All commands sent. Reset the sequence.
                IRCommandStep = 0; 
                break;
        }
        
        previousIRMillis = millis(); // Reset the timer for the next step
    }
    // --- End Non-Blocking IR Transmission Sequence ---

}
    

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    TimeSyncSuccessBool = false;
    return;
  }
  else{
    TimeSyncSuccessBool = true;
  }  
}

void WiFiEvent(WiFiEvent_t event) {
  //courtesy of Google Gemini
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
      Serial.println("WiFi lost connection, attempting to reconnect...");
      // *** THE NON-BLOCKING RECONNECTION STEP ***
      // Call WiFi.begin() to reconnect. The ESP32 handles the retries.
      WiFi.begin(ssid, password);
      TimeSyncSuccessBool = false; // Indicate time may be stale
      Wifi_State = "disconnected";
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      // Re-initialize and get the time after a successful connection/reconnection
      configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
      printLocalTime();
      Wifi_State = "GOT IP";
      // Restart the server if it was stopped (optional, depends on AsyncWebServer library behavior)
      // server.begin();
      break;
    default:
      break;
  }
}