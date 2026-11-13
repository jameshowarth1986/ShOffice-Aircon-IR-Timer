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

// --- 4b. 24-Hour Temperature History Buffer (96 points = 24h @ 15-min intervals) ---
//#define USE_DUMMY_HISTORY  // <--- COMMENT OUT THIS LINE TO DISABLE DUMMY DATA

const int HISTORY_SIZE = 96;
struct TempRecord {
  char timeStr[6]; // "HH:MM"
  float temp;
};
TempRecord tempHistory[HISTORY_SIZE];
int historyHead = 0;          // Points to next write index
int historyCount = 0;         // Number of valid records stored (0 to 96)
int lastLoggedMinute = -1;    // Prevents duplicate logging within the same minute

// --- 5. Parameters & Web Input Storage ---
char myParameter1[4] = "xx";  // Hour
char myParameter2[4] = "xx";  // Minute
char myParameter3[4] = "22";  // Temp
char myParameter4[4] = "xx";  // Mode
char myParameter5[4] = "xx";  // Run

// --- 6. Timing & IR Sequence ---
struct tm timenow;                                  
unsigned long TimeIntervalPrevious = 0;             
const int TimeSyncInterval = (1000 * 60 * 60 * 6);  // 6 Hours

unsigned long previousIRMillis = 0;  
const long IRInterval = 2000;        
int IRCommandStep = 0;               

unsigned long OLEDUpdateIntervalPrevious = 0;  
const int OLEDUpdateInterval = 500;

const long wifiCheckInterval = 10000;  


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
void logTemperatureHistory();

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

// Main Dashboard HTML (Light Mode)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>AC Controller</title>
  <style>
    :root {
      --bg: #F4F6F9;
      --card-bg: #FFFFFF;
      --text-main: #1E293B;
      --text-muted: #64748B;
      --border: #E2E8F0;
      --accent-cool: #0284C7;
      --accent-heat: #EA580C;
      --accent-green: #16A34A;
      --danger: #EF4444;
      --shadow-sm: 0 2px 4px rgba(0,0,0,0.02), 0 1px 2px rgba(0,0,0,0.03);
      --shadow-md: 0 10px 25px -5px rgba(0,0,0,0.05), 0 8px 10px -6px rgba(0,0,0,0.01);
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; }
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
    .footer-links { display: flex; justify-content: space-between; margin-top: 18px; padding-top: 14px; border-top: 1px solid var(--border); }
    .footer-links a { color: var(--accent-cool); font-size: 0.9rem; font-weight: 700; text-decoration: none; }
    .footer-links a.muted { color: var(--text-muted); font-weight: 600; }
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
      <div class="footer-links">
        <a href="/history">24h Temp History &rarr;</a>
        <a href="/" class="muted">Refresh Status</a>
      </div>
    </div>
  </div>
</body>
</html>
)rawliteral";

// Interactive Temperature History Page (Chart.js + Offline Table Fallback)
// Interactive Temperature History Page (Chart.js + Offline Table Fallback)
const char history_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>24h Temperature History</title>
  <script src="https://cdn.jsdelivr.net/npm/chart.js@4.4.4/dist/chart.umd.min.js"></script>
  <style>
    :root {
      --bg: #F4F6F9;
      --card-bg: #FFFFFF;
      --text-main: #1E293B;
      --text-muted: #64748B;
      --border: #E2E8F0;
      --accent: #0284C7;
      --shadow-md: 0 10px 25px -5px rgba(0,0,0,0.05), 0 8px 10px -6px rgba(0,0,0,0.01);
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; }
    body {
      background-color: var(--bg);
      color: var(--text-main);
      min-height: 100vh;
      padding: 16px 12px;
    }
    .container {
      width: 100%%;
      max-width: 480px;
      margin: 0 auto;
    }
    .card {
      background: var(--card-bg);
      border-radius: 22px;
      padding: 18px;
      box-shadow: var(--shadow-md);
      border: 1px solid var(--border);
    }
    .header { margin-bottom: 14px; }
    .header h1 { font-size: 1.3rem; font-weight: 800; }
    .subtitle { font-size: 0.8rem; color: var(--text-muted); margin-top: 2px; }
    .legend-row {
      display: flex;
      flex-wrap: wrap;
      gap: 12px;
      margin-bottom: 14px;
      font-size: 0.76rem;
      font-weight: 600;
      color: var(--text-muted);
    }
    .legend-item { display: flex; align-items: center; gap: 5px; }
    .dot-temp { width: 10px; height: 10px; border-radius: 50%%; background: #0284C7; }
    .line-target { width: 14px; height: 0; border-top: 2px dashed #EA580C; }
    .line-start { width: 0; height: 12px; border-left: 2px dashed #16A34A; }
    .stats-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      gap: 8px;
      margin-bottom: 16px;
    }
    .stat-box {
      background: #F8FAFC;
      border: 1px solid var(--border);
      border-radius: 12px;
      padding: 10px 6px;
      text-align: center;
    }
    .stat-label {
      font-size: 0.68rem;
      font-weight: 700;
      color: var(--text-muted);
      text-transform: uppercase;
      letter-spacing: 0.4px;
    }
    .stat-val {
      font-size: 1.1rem;
      font-weight: 800;
      color: var(--text-main);
      margin-top: 3px;
    }
    .chart-box {
      position: relative;
      width: 100%%;
      height: 270px;
      background: #F8FAFC;
      border: 1px solid var(--border);
      border-radius: 16px;
      padding: 10px 8px 6px 4px;
    }
    .empty-msg {
      height: 100%%;
      display: flex;
      align-items: center;
      justify-content: center;
      color: var(--text-muted);
      font-size: 0.85rem;
      font-weight: 600;
      text-align: center;
      padding: 20px;
      line-height: 1.4;
    }
    .offline-banner {
      background: #FEF3C7;
      color: #92400E;
      font-size: 0.78rem;
      font-weight: 600;
      padding: 8px 12px;
      border-radius: 10px;
      margin-bottom: 10px;
      text-align: center;
    }
    .table-wrap {
      max-height: 240px;
      overflow-y: auto;
      border: 1px solid var(--border);
      border-radius: 12px;
      background: #FFFFFF;
    }
    table { width: 100%%; border-collapse: collapse; font-size: 0.88rem; }
    th, td { padding: 9px 14px; text-align: left; border-bottom: 1px solid var(--border); }
    th { background: #F1F5F9; font-weight: 700; position: sticky; top: 0; }
    .actions {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-top: 16px;
    }
    .btn {
      padding: 13px 10px;
      border: none;
      border-radius: 12px;
      font-size: 0.9rem;
      font-weight: 700;
      cursor: pointer;
      text-align: center;
      text-decoration: none;
      display: flex;
      align-items: center;
      justify-content: center;
    }
    .btn-primary { background: var(--text-main); color: #FFFFFF; }
    .btn-secondary { background: #E2E8F0; color: var(--text-main); }
  </style>
</head>
<body>
  <div class="container">
    <div class="card">
      <div class="header">
        <h1>Temperature History</h1>
        <div class="subtitle">Tap or drag across the graph for exact readings</div>
      </div>

      <div class="legend-row">
        <div class="legend-item"><span class="dot-temp"></span> Measured</div>
        <div class="legend-item"><span class="line-target"></span> Target (<span id="leg-target">--</span>)</div>
        <div class="legend-item" id="leg-start-wrap" style="display:none;"><span class="line-start"></span> Start (<span id="leg-start">--</span>)</div>
      </div>

      <div class="stats-grid">
        <div class="stat-box">
          <div class="stat-label">Current</div>
          <div class="stat-val" id="stat-curr">--</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">24h Low</div>
          <div class="stat-val" id="stat-min">--</div>
        </div>
        <div class="stat-box">
          <div class="stat-label">24h High</div>
          <div class="stat-val" id="stat-max">--</div>
        </div>
      </div>

      <div class="chart-box" id="chartContainer">
        <canvas id="tempChart"></canvas>
      </div>

      <div class="actions">
        <a href="/" class="btn btn-primary">&larr; Controls</a>
        <button onclick="loadHistory()" class="btn btn-secondary">Refresh</button>
      </div>
    </div>
  </div>

  <script>
    let chartInstance = null;

    const findStartIndex = (labels, startTime) => {
      if (!startTime || labels.length === 0) return -1;
      const exact = labels.indexOf(startTime);
      if (exact !== -1) return exact;

      const [sh, sm] = startTime.split(':').map(Number);
      const targetMins = sh * 60 + sm;
      let bestIdx = -1;
      let bestDiff = 9999;

      labels.forEach((lbl, idx) => {
        const [lh, lm] = lbl.split(':').map(Number);
        const diff = Math.abs((lh * 60 + lm) - targetMins);
        if (diff <= 15 && diff < bestDiff) {
          bestDiff = diff;
          bestIdx = idx;
        }
      });
      return bestIdx;
    };

    const renderFallbackTable = (payload) => {
      const container = document.getElementById('chartContainer');
      container.style.height = 'auto';
      container.style.padding = '10px';

      const pts = (payload.points || []).slice().reverse();
      if (pts.length === 0) {
        container.innerHTML = '<div class="empty-msg">No 15-minute intervals logged yet.<br>Readings record at :00, :15, :30, and :45.</div>';
        return;
      }

      let rows = pts.map(p => `<tr><td>${p.t}</td><td><strong>${p.v.toFixed(1)}°C</strong></td></tr>`).join('');
      container.innerHTML = `
        <div class="offline-banner">Offline mode — showing recorded log table</div>
        <div class="table-wrap">
          <table>
            <thead><tr><th>Time</th><th>Temperature</th></tr></thead>
            <tbody>${rows}</tbody>
          </table>
        </div>`;
    };

    const renderInteractiveChart = (payload) => {
      document.getElementById('stat-curr').textContent = payload.current.toFixed(1) + '°C';
      document.getElementById('leg-target').textContent = payload.target + '°C';

      const startWrap = document.getElementById('leg-start-wrap');
      if (payload.systemRun) {
        startWrap.style.display = 'flex';
        document.getElementById('leg-start').textContent = payload.startTime;
      } else {
        startWrap.style.display = 'none';
      }

      const pts = payload.points || [];
      if (pts.length === 0) {
        document.getElementById('stat-min').textContent = '--';
        document.getElementById('stat-max').textContent = '--';
        document.getElementById('chartContainer').innerHTML =
          '<div class="empty-msg">No 15-minute intervals logged yet.<br>Readings record at :00, :15, :30, and :45.</div>';
        return;
      }

      const temps = pts.map(p => p.v);
      document.getElementById('stat-min').textContent = Math.min(...temps).toFixed(1) + '°C';
      document.getElementById('stat-max').textContent = Math.max(...temps).toFixed(1) + '°C';

      if (typeof Chart === 'undefined') {
        renderFallbackTable(payload);
        return;
      }

      const container = document.getElementById('chartContainer');
      if (!document.getElementById('tempChart')) {
        container.innerHTML = '<canvas id="tempChart"></canvas>';
      }

      const labels = pts.map(p => p.t);
      const targetSeries = pts.map(() => payload.target);
      const startIdx = payload.systemRun ? findStartIndex(labels, payload.startTime) : -1;

      const startLinePlugin = {
        id: 'startLine',
        afterDatasetsDraw(chart) {
          if (startIdx < 0) return;
          const { ctx, chartArea: { top, bottom }, scales: { x } } = chart;
          const xPos = x.getPixelForValue(startIdx);
          if (xPos === undefined) return;

          ctx.save();
          ctx.beginPath();
          ctx.setLineDash([5, 4]);
          ctx.lineWidth = 2;
          ctx.strokeStyle = '#16A34A';
          ctx.moveTo(xPos, top);
          ctx.lineTo(xPos, bottom);
          ctx.stroke();

          ctx.fillStyle = '#16A34A';
          ctx.font = '700 10px -apple-system, sans-serif';
          ctx.textAlign = xPos > chart.width - 65 ? 'right' : 'left';
          ctx.fillText('Start ' + payload.startTime, xPos + (xPos > chart.width - 65 ? -5 : 5), top + 12);
          ctx.restore();
        }
      };

      if (chartInstance) {
        chartInstance.destroy();
      }

      const ctx = document.getElementById('tempChart').getContext('2d');
      chartInstance = new Chart(ctx, {
        type: 'line',
        data: {
          labels: labels,
          datasets: [
            {
              label: 'Office Temp',
              data: temps,
              borderColor: '#0284C7',
              backgroundColor: 'rgba(2, 132, 199, 0.14)',
              borderWidth: 2.5,
              tension: 0.3,
              fill: true,
              pointRadius: pts.length > 48 ? 2 : 3.5,
              pointHoverRadius: 6,
              pointBackgroundColor: '#0284C7'
            },
            {
              label: 'Target Setpoint',
              data: targetSeries,
              borderColor: '#EA580C',
              borderWidth: 1.8,
              borderDash: [6, 4],
              pointRadius: 0,
              pointHoverRadius: 0,
              fill: false
            }
          ]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          interaction: {
            mode: 'index',
            intersect: false
          },
          plugins: {
            legend: { display: false },
            tooltip: {
              backgroundColor: '#1E293B',
              padding: 10,
              cornerRadius: 10,
              callbacks: {
                label: (ctx) => `${ctx.dataset.label}: ${ctx.parsed.y.toFixed(1)}°C`
              }
            }
          },
          scales: {
            x: {
              grid: { display: false },
              ticks: {
                color: '#64748B',
                font: { size: 10, weight: '600' },
                maxTicksLimit: 6,
                maxRotation: 0
              }
            },
            y: {
              grid: { color: '#E2E8F0' },
              ticks: {
                color: '#64748B',
                font: { size: 10, weight: '600' },
                callback: (val) => val.toFixed(1) + '°'
              }
            }
          }
        },
        plugins: [startLinePlugin]
      });
    };

    const loadHistory = () => {
      fetch('/history-data')
        .then(r => r.json())
        .then(data => renderInteractiveChart(data))
        .catch(() => {
          document.getElementById('chartContainer').innerHTML =
            '<div class="empty-msg">Failed to connect to ESP32.</div>';
        });
    };

    loadHistory();
  </script>
</body>
</html>
)rawliteral";



const char success_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Settings Updated</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <style>
    body { font-family: -apple-system, sans-serif; background: #F4F6F9; text-align: center; margin-top: 50px; }
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
  <style>
    body { font-family: -apple-system, sans-serif; background: #F4F6F9; text-align: center; margin-top: 50px; }
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
  <style>
    body { font-family: -apple-system, sans-serif; background: #F4F6F9; text-align: center; margin-top: 50px; }
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

#ifdef USE_DUMMY_HISTORY
void populateDummyHistory() {
  for (int i = 0; i < HISTORY_SIZE; i++) {
    int totalMinutes = i * 15;
    int h = totalMinutes / 60;
    int m = totalMinutes % 60;
    snprintf(tempHistory[i].timeStr, sizeof(tempHistory[i].timeStr), "%02d:%02d", h, m);

    // Smooth curve: 10.0°C at 00:00 -> 22.0°C at 12:00 (index 48) -> 10.0°C at 23:45
    float progress = (i <= 48) ? (i / 48.0f) : ((96 - i) / 48.0f);
    tempHistory[i].temp = 10.0f + (12.0f * progress);
  }
  historyHead = 0;
  historyCount = HISTORY_SIZE;
  Serial.println("Dummy 24h temperature history loaded.");
}
#endif

void setup() {
  Serial.begin(115200);

  sprintf(myParameter1, "%02d", AC_Turn_On_Time_Hour);
  sprintf(myParameter2, "%02d", AC_Turn_On_Time_Minute);

  WiFi.onEvent(WiFiEvent);  

  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);  
  WiFi.begin(ssid, password);

  // Main Dashboard Route
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  // New History Graph Page Route
  server.on("/history", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", history_html);
  });

  // JSON Endpoint serving the 96-point circular buffer + Target & Schedule metadata
  server.on("/history-data", HTTP_GET, [](AsyncWebServerRequest* request) {
    char startStr[6];
    snprintf(startStr, sizeof(startStr), "%02d:%02d", AC_Turn_On_Time_Hour, AC_Turn_On_Time_Minute);

    String json;
    json.reserve(2600);
    json += "{\"current\":";
    json += String(currentTemp, 1);
    json += ",\"target\":";
    json += String(TargetTemperature);
    json += ",\"startTime\":\"";
    json += String(startStr);
    json += "\",\"systemRun\":";
    json += (runSystem ? "true" : "false");
    json += ",\"points\":[";

    int startIdx = (historyCount == HISTORY_SIZE) ? historyHead : 0;
    for (int i = 0; i < historyCount; i++) {
      int idx = (startIdx + i) % HISTORY_SIZE;
      if (i > 0) json += ",";
      json += "{\"t\":\"";
      json += tempHistory[idx].timeStr;
      json += "\",\"v\":";
      json += String(tempHistory[idx].temp, 1);
      json += "}";
    }

    json += "]}";
    request->send(200, "application/json", json);
  });

  server.on("/trigger", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (IRCommandStep == 0) {  
      IRCommandStep = 1;
      previousIRMillis = millis();  
      Serial.println("Manual IR Triggered via Web");
      request->send(200, "text/html", sequence_initiated);
    } else {
      request->send(200, "text/html", "Sequence already in progress.<br><a href=\"/\">Return</a>");
    }
  });

  server.on("/off", HTTP_GET, [](AsyncWebServerRequest* request) {
    MyLG_Aircondition.sendCommandAndParameter(LG_COMMAND_OFF, 0); 
    Serial.println("Manual IR OFF Triggered via Web");
    request->send_P(200, "text/html", power_off_cmd_sent); 
  });

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
    
    for(int i = 0; i < 7; i++) {
      activeDays[i] = false;
    }
    
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

  #ifdef USE_DUMMY_HISTORY
  populateDummyHistory();
  #endif

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

  // Log temperature history at :00, :15, :30, and :45 past the hour
  logTemperatureHistory();

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

void logTemperatureHistory() {
#ifndef USE_DUMMY_HISTORY
  // Wait until NTP time is synced and a valid temperature reading exists
  if (!TimeSyncSuccessBool || currentTemp == 0 || currentTemp == 99) return;

  struct tm now;
  if (!getLocalTime(&now, 0)) return;

  // Check if current minute is 0, 15, 30, or 45 and hasn't been logged yet this minute
  if ((now.tm_min % 15 == 0) && (now.tm_min != lastLoggedMinute)) {
    lastLoggedMinute = now.tm_min;

    snprintf(tempHistory[historyHead].timeStr, sizeof(tempHistory[historyHead].timeStr), "%02d:%02d", now.tm_hour, now.tm_min);
    tempHistory[historyHead].temp = currentTemp;

    historyHead = (historyHead + 1) % HISTORY_SIZE;
    if (historyCount < HISTORY_SIZE) {
      historyCount++;
    }

    Serial.printf("Logged History [%s]: %.1f C (Total points: %d)\n", tempHistory[(historyHead + HISTORY_SIZE - 1) % HISTORY_SIZE].timeStr, currentTemp, historyCount);
  }
#endif
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