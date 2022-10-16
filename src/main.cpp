// Copyright 2022 Michael Harwerth miq1 _ at _ gmx _ dot _ de

#include <Arduino.h>
#include <Version.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESPAsyncTCP.h>
#include <ESP8266mDNS.h>
#include <LittleFS.h>
#include <ESP8266WebServer.h>
#include "DHTesp.h"
#include "Blinker.h"
#include "Buttoner.h"
#include "RingBuf.h"
#include "ModbusClientTCPAsync.h"
#include "ModbusServerTCPAsync.h"
#undef LOCAL_LOG_LEVEL
#define LOCAL_LOG_LEVEL LOG_LEVEL_VERBOSE
#include "Logging.h"

// To do:
// WiFi
// Modbus
// Config

// Pin definitions
#define SENSOR_0 D2
#define SENSOR_1 D1
#define SIGNAL_LED D6
#define S1STATUS_LED D0
#define S2STATUS_LED D4
#define TARGET_LED D5
#define SWITCH_PIN D3
#define TARGET_PIN D7

IPAddress myIP;               // local IP address
char AP_SSID[24];             // AP SSID for config mode
// Runtime modes
enum MODE_T : uint8_t { RUN, CONFIG, MANUAL };
MODE_T mode = RUN;
uint16_t runTime = 0;

// NTP definitions
#ifndef MY_NTP_SERVER
#define MY_NTP_SERVER "pool.ntp.org"
#endif
#ifndef MY_TZ
#define MY_TZ "CET"
#endif

// Milliseconds between measurements
uint32_t INTERVAL_DHT = 20000;

// Web server definitions
ESP8266WebServer HTMLserver(80);
#define SET_JS "/set.js"
#define CONFIG_HTML "/config.html"
#define DEVICE_HTML "/device.html"
#define SETTINGS "/settings.bin"

// Target address for Modbus device
struct ModbusTarget {
  uint32_t ip;
  uint16_t port;
  uint8_t serverID;
  uint8_t isValid;
  ModbusTarget() : ip(0), port(0), serverID(0), isValid(0) {}
};

// Switch hysteresis: only if a sequence of n measurements is identical, a switch action is considered
uint16_t Hysteresis = 0xAAAA;                               // holds last 16 results
uint16_t HYSTERESIS_MASK = 0x000F;                          // Bit mask to check last n measurements

// Target tracking
uint16_t targetHealth = 0;
bool switchedON = false;

// Blue status LED will blink differently if target socket is switched on
Blinker signalLED(SIGNAL_LED);
Blinker S1LED(S1STATUS_LED);
Blinker S2LED(S2STATUS_LED);
Blinker targetLED(TARGET_LED);

// Blink pattern for "target ON"
const uint16_t TARGET_ON_BLINK(0xA800);
// Blink pattern for "target OF"
const uint16_t TARGET_OFF_BLINK(0x1000);
// Wait for initial button press
const uint16_t KNOB_BLINK(0x1111);
// CONFIG mode:
const uint16_t CONFIG_BLINK(0xCCC0);
// Wifi connect mode:
const uint16_t WIFI_BLINK(0xFF00);
// Manual mode
const uint16_t MANUAL_BLINK(0xFFFF);
// Device ignored
const uint16_t DEVICE_IGNORED(0x0000);
// Device OK
const uint16_t DEVICE_OK(0xFFFF);
// Device error
const uint16_t DEVICE_ERROR_BLINK(0xAAAA);

// Switch to trigger restarts etc.
Buttoner tSwitch(SWITCH_PIN, LOW);

// Two DHT sensors. In setup() will be found out if both are connected
struct mySensor {
  DHTesp sensor;
  TempAndHumidity th;
  float dewPoint;
  Blinker& statusLED;
  uint16_t healthTracker;
  uint8_t sensor01;
  bool lockFlag;
  mySensor(Blinker& sLED, uint8_t whichOne) : statusLED(sLED), healthTracker(0), sensor01(whichOne), lockFlag(false) { }
};
mySensor DHT0(S1LED, 0);
mySensor DHT1(S2LED, 1);

// Choice list for target and sensor devices
enum DEVICEMODE : uint8_t { DEV_NONE=0, DEV_LOCAL, DEV_MODBUS };
enum DEVICECOND : uint8_t { DEVC_NONE=0, DEVC_LESS, DEVC_GREATER };
class PORTNUM {
protected:
  uint16_t value;
public:
  PORTNUM() : value(0) {}
  explicit PORTNUM(uint16_t v) : value(v) {}
  operator uint16_t() { return value; }
  uint16_t operator=(uint16_t v) { return (value = v); }
};
class SIDTYPE {
protected:
  uint8_t value;
public:
  SIDTYPE() : value(0) {}
  explicit SIDTYPE(uint8_t v) : value(v) {}
  operator uint8_t() { return value; }
  uint8_t operator=(uint8_t v) { return (value = v); }
};

// Settings data
const uint16_t MAGICVALUE(0x4715);
const uint8_t STRINGPARMLENGTH(32);
const uint8_t CONFIGPARAMS(48);
struct SetData {
  uint16_t magicValue;                   // 0x4712 upon successful initialization
  char deviceName[STRINGPARMLENGTH];     // CV0 Name of this device for mDNS, OTA etc.
  char WiFiSSID[STRINGPARMLENGTH];       // CV1 SSID of local WiFi
  char WiFiPASS[STRINGPARMLENGTH];       // CV2 Password for local WiFi
  char OTAPass[STRINGPARMLENGTH];        // CV3 OTA password
  bool masterSwitch;                     // CV4 Functions on/off main switch
  DEVICEMODE Target;                     // CV7 0=none, 1=local, 2=Modbus
  uint8_t hystSteps;                     // CV5 hysteresis step count 0..15 (16, 1..15)
  uint16_t measuringInterval;            // CV6 time between measurements in seconds
  IPAddress targetIP;                    // CV8..CV11 
  PORTNUM targetPort;                    // CV12 target Modbus port number
  SIDTYPE targetSID;                     // CV13 target Modbus server ID
  struct SensorData {
    DEVICEMODE type;                     // S0:CV14 S1:CV28 0=none, 1=local, 2=Modbus
    IPAddress IP;                        // S0:CV15..CV18 S1:CV29..CV32
    PORTNUM port;                        // S0:CV19 S1:CV33 Sensor Modbus port number
    SIDTYPE SID;                         // S0:CV20 S1:CV34 Sensor Modbus server ID
    bool slot;                           // S0:CV21 S1:CV35 Sensor Modbus slot: false=slot 1, true=slot 2
    DEVICECOND TempMode;                 // S0:CV22 S1:CV36 Sensor temperature condition 0:ignore, 1:<, 2:>
    float Temp;                          // S0:CV23 S1:CV37 Sensor condition temperature value
    DEVICECOND HumMode;                  // S0:CV24 S1:CV38 Sensor humidity condition 0:ignore, 1:<, 2:>
    float Hum;                           // S0:CV25 S1:CV39 Sensor condition humidity value
    DEVICECOND DewMode;                  // S0:CV26 S1:CV40 Sensor dew point condition 0:ignore, 1:<, 2:>
    float Dew;                           // S0:CV27 S1:CV41 Sensor condition dew point value
  } sensor[2];
  DEVICECOND TempDiff;                   // CV42 (S0 - S1) temperature condition 0:ignore, 1:<, 2:>
  float Temp;                            // CV43 (S0 - S1) condition temperature value
  DEVICECOND HumDiff;                    // CV44 (S0 - S1) humidity condition 0:ignore, 1:<, 2:>
  float Hum;                             // CV45 (S0 - S1) condition humidity value
  DEVICECOND DewDiff;                    // CV46 (S0 - S1) dew point condition 0:ignore, 1:<, 2:>
  float Dew;                             // CV47 (S0 - S1) condition dew point value
  uint16_t restarts;                     // number of reboots
  SetData() {
    magicValue = 0;
  }
} settings;

// Write both SETTINGS and SET_JS files with current settings data
// Some helper functions first
void writeSetting(Print& st, const char *header, uint8_t num, uint8_t target) {
  st.printf("%s.CV%d.value=\"%u\";\n", header, num, target);
}

void writeSetting(Print& st, const char *header, uint8_t num, uint16_t target) {
  st.printf("%s.CV%d.value=\"%u\";\n", header, num, target);
}

void writeSetting(Print& st, const char *header, uint8_t num, DEVICEMODE target) {
  st.printf("%s.CV%d.value=\"%u\";\n", header, num, target);
}

void writeSetting(Print& st, const char *header, uint8_t num, DEVICECOND target) {
  st.printf("%s.CV%d.value=\"%u\";\n", header, num, target);
}

void writeSetting(Print& st, const char *header, uint8_t num, PORTNUM target) {
  st.printf("%s.CV%d.value=\"%u\";\n", header, num, uint16_t(target));
  st.printf("%s.CV%d.min=\"1\";\n", header, num);
  st.printf("%s.CV%d.max=\"65535\";\n", header, num);
  st.printf("%s.CV%d.size=\"9\";\n", header, num);
  st.printf("%s.CV%d.step=\"1\";\n", header, num);
}

void writeSetting(Print& st, const char *header, uint8_t num, SIDTYPE target) {
  st.printf("%s.CV%d.value=\"%u\";\n", header, num, uint8_t(target));
  st.printf("%s.CV%d.min=\"1\";\n", header, num);
  st.printf("%s.CV%d.max=\"247\";\n", header, num);
  st.printf("%s.CV%d.size=\"5\";\n", header, num);
  st.printf("%s.CV%d.step=\"1\";\n", header, num);
}

void writeSetting(Print& st, const char *header, uint8_t num, float target) {
  st.printf("%s.CV%d.value=\"%.1f\";\n", header, num, target);
  st.printf("%s.CV%d.step=\"0.1\";\n", header, num);
}

void writeSetting(Print& st, const char *header, uint8_t num, char* target) {
  st.printf("%s.CV%d.value=\"%s\";\n", header, num, target);
  st.printf("%s.CV%d.size=\"%d\";\n", header, num, STRINGPARMLENGTH - 1);
  st.printf("%s.CV%d.maxlength=\"%d\";\n", header, num, STRINGPARMLENGTH - 1);
}

void writeSetting(Print& st, const char *header, uint8_t num, IPAddress target) {
  for (uint8_t i = 0; i < 4; i++) {
    st.printf("%s.CV%d.value=\"%u\";\n", header, num + i, target[i]);
    st.printf("%s.CV%d.min=\"0\";\n", header, num + i);
    st.printf("%s.CV%d.max=\"255\";\n", header, num + i);
    st.printf("%s.CV%d.size=\"5\";\n", header, num + i);
    st.printf("%s.CV%d.step=\"1\";\n", header, num + i);
  }
}

int writeSettings() {
  int rc = 0;
  // First write binary file SETTINGS
  File sF = LittleFS.open(SETTINGS, "w");
  if (sF) {
    sF.write((char *)&settings, sizeof(SetData));
    sF.close();
    // Next we need to write SET_JS to be used on the HTML config page
    File sJ = LittleFS.open(SET_JS, "w");
    if (sJ) {
      // Write function header
      sJ.println("function setValues() {");
      // One by one write the settings values
      const char *head = "  document.F";
      writeSetting(sJ, head,  0, settings.deviceName);
      writeSetting(sJ, head,  1, settings.WiFiSSID);
      writeSetting(sJ, head,  2, settings.WiFiPASS);
      writeSetting(sJ, head,  3, settings.OTAPass);
      writeSetting(sJ, head,  4, (uint8_t)(settings.masterSwitch ? 1 : 0));
      writeSetting(sJ, head,  5, settings.hystSteps);
      writeSetting(sJ, head,  6, settings.measuringInterval);
      writeSetting(sJ, head,  7, settings.Target);
      writeSetting(sJ, head,  8, settings.targetIP);
      writeSetting(sJ, head, 12, settings.targetPort);
      writeSetting(sJ, head, 13, settings.targetSID);
      writeSetting(sJ, head, 14, settings.sensor[0].type);
      writeSetting(sJ, head, 15, settings.sensor[0].IP);
      writeSetting(sJ, head, 19, settings.sensor[0].port);
      writeSetting(sJ, head, 20, settings.sensor[0].SID);
      writeSetting(sJ, head, 21, (uint8_t)(settings.sensor[0].slot ? 2 : 1));
      writeSetting(sJ, head, 22, settings.sensor[0].TempMode);
      writeSetting(sJ, head, 23, settings.sensor[0].Temp);
      writeSetting(sJ, head, 24, settings.sensor[0].HumMode);
      writeSetting(sJ, head, 25, settings.sensor[0].Hum);
      writeSetting(sJ, head, 26, settings.sensor[0].DewMode);
      writeSetting(sJ, head, 27, settings.sensor[0].Dew);
      writeSetting(sJ, head, 28, settings.sensor[1].type);
      writeSetting(sJ, head, 29, settings.sensor[1].IP);
      writeSetting(sJ, head, 33, settings.sensor[1].port);
      writeSetting(sJ, head, 34, settings.sensor[1].SID);
      writeSetting(sJ, head, 35, (uint8_t)(settings.sensor[1].slot ? 2 : 1));
      writeSetting(sJ, head, 36, settings.sensor[1].TempMode);
      writeSetting(sJ, head, 37, settings.sensor[1].Temp);
      writeSetting(sJ, head, 38, settings.sensor[1].HumMode);
      writeSetting(sJ, head, 39, settings.sensor[1].Hum);
      writeSetting(sJ, head, 40, settings.sensor[1].DewMode);
      writeSetting(sJ, head, 41, settings.sensor[1].Dew);
      writeSetting(sJ, head, 42, settings.TempDiff);
      writeSetting(sJ, head, 43, settings.Temp);
      writeSetting(sJ, head, 44, settings.HumDiff);
      writeSetting(sJ, head, 45, settings.Hum);
      writeSetting(sJ, head, 46, settings.DewDiff);
      writeSetting(sJ, head, 47, settings.Dew);
      // Write function footer
      sJ.println("}");
      sJ.close();
    } else {
      Serial.printf("Could not write '" SET_JS "'");
      rc = 2;
    }
  } else {
    Serial.printf("Could not write '" SETTINGS "'");
    rc = 1;
  }
  return rc;
}

// Number of event slots
const uint8_t MAXEVENT(40);
// Define the event types
enum S_EVENT : uint8_t  { 
  NO_EVENT=0, DATE_CHANGE,
  BOOT_DATE, BOOT_TIME, 
  MASTER_ON, MASTER_OFF,
  TARGET_ON, TARGET_OFF,
};
const char *eventname[] = { 
  "no event", "date change", "boot date", "boot time", 
  "MASTER on", "MASTER off", 
  "target on", "target off", 
};
// Allocate event buffer
RingBuf<uint16_t> events(MAXEVENT);

// Server for own data
ModbusServerTCPasync MBserver;
// Client to access switch and sensor sources
ModbusClientTCPasync MBclient(IPAddress(0, 0, 0, 0), 502);

// Modbus register map
// *******************************

// checkSensor: test if physical sensor is functional
int checkSensor(mySensor& ms, const char *label) {
  int rc = -1;

  // advance health tracker
  ms.healthTracker <<= 1;
  // The DHT lib will return nan values if no sensor is present
  if (!isnan(ms.sensor.getTemperature())) {
    // We got a value - sensor seems to be functional
    LOG_I("Sensor %s ok.\n", label);
    ms.healthTracker |= 1;
    // Light upper status LED
    ms.statusLED.start(DEVICE_OK);
    rc = 0;
  } else {
    // We caught a nan.
    LOG_E("Sensor %s: error %s\n", label, ms.sensor.getStatusString());
    // Turn off status LED
    ms.statusLED.start(DEVICE_IGNORED);
  }
  return rc;
}

// takeMeasurement: get temperature, humidity and dew point for a sensor
// If no sensor is connected, try to get data from Modbus source
bool takeMeasurement(mySensor& ms, const char *label) {
  bool rc = false;
  SetData::SensorData& sd = settings.sensor[ms.sensor01];

  // Physical sensor configured?
  if (sd.type == DEV_LOCAL) {
    ms.healthTracker <<= 1;
    // Yes. get the data
    ms.th = ms.sensor.getTempAndHumidity();
    ms.dewPoint = ms.sensor.computeDewPoint(ms.th.temperature, ms.th.humidity);
    // update status
    if (!isnan(ms.th.temperature)) {
      rc = true;
      ms.healthTracker |= 1;
      ms.statusLED.start(DEVICE_OK);
    } else {
      ms.statusLED.start(DEVICE_ERROR_BLINK);
    }
  // No, but a Modbus source perhaps?
  } else if (sd.type == DEV_MODBUS) {
    // Yes. Check Modbus
    // Is Modbus address set?
    if (sd.IP && sd.port && sd.SID) {
      // Yes. Set target and send a request
      // This will be asynchronous, so we may use the previous data for now.
      MBclient.connect(sd.IP, sd.port);
      Error e = MBclient.addRequest((uint32_t)(0x1008 | ms.sensor01), 
        sd.SID, 
        READ_HOLD_REGISTER,
        (uint16_t)(sd.slot ? 8 : 2),  // Register address slot 1:2, slot 2:2 + 6 = 8
        (uint16_t)6);
      if (e != SUCCESS) {
        ModbusError me(e);
        LOG_E("Error requesting sensor %d - %s\n", ms.sensor01, (const char *)me);
      } else {
        rc = true;
      }
    } 
    // Request sent?
    if (!rc) {
      // No, count as failure
      ms.healthTracker <<= 1;
      ms.statusLED.start(DEVICE_ERROR_BLINK);
    }
  }
  return rc;
}

// registerEvent: append another event to the buffer
void registerEvent(S_EVENT ev) {
  // We will need date and/or time
  time_t now = time(NULL);
  tm tm;
  localtime_r(&now, &tm);           // update the structure tm with the current time
  uint16_t eventWord = NO_EVENT << 11;
  uint8_t hi = 0;
  uint8_t lo = 0;
  
 // Set the event word
  if (ev == BOOT_DATE || ev == DATE_CHANGE) {
    // Need the date
    hi = tm.tm_mday & 0x1F;
    lo = (tm.tm_mon + 1) & 0x3F;
  } else {
    // Need the time
    hi = tm.tm_hour & 0x1F;
    lo = tm.tm_min & 0x3F;
  }
  eventWord = ((ev & 0x1F) << 11) | (hi << 6) | lo;

  // Prevent duplicates - last event must differ
  if (events[events.size() - 1] != eventWord) {
    // Push the word
    events.push_back(eventWord);
  }
}

// -----------------------------------------------------------------------------
// Setup WiFi in RUN mode
// -----------------------------------------------------------------------------
void wifiSetup(const char *hostname) {
  // Start WiFi connection blinking pattern
  signalLED.start(WIFI_BLINK, 100);
  // Set WIFI module to STA mode
  WiFi.mode(WIFI_STA);

  // If we already have a device name, use it as hostname as well
  if (*hostname) {
    WiFi.hostname(hostname);
  }

  // Connect
  // WiFi.begin(settings.WiFiSSID, settings.WiFiPASS);
  WiFi.begin(MYSSID, MYPASS);

  // Wait for connection. ==> We will hang here in RUN mode forever without a WiFi!
  while (WiFi.status() != WL_CONNECTED) {
    signalLED.update();
    delay(50);
  }

  myIP = WiFi.localIP();
  
  LOG_I("IP=%d.%d.%d.%d\n", myIP[0], myIP[1], myIP[2], myIP[3]);

  // Start mDNS service
  if (*hostname) {
    MDNS.begin(hostname);
  }

  // Connected! Stop blinking
  signalLED.stop();
}

// Response handler for Modbus client
void handleResponse(ModbusMessage response, uint32_t token) {
  Error e = response.getError();

  LOG_V("Token %04X\n", token);
  HEXDUMP_V("Response ", response.data(), response.size());

  // Did we get an error?
  if (e != SUCCESS) {
    // Yes. Report it
    ModbusError me(e);
    LOG_E("Error response for request %04X: %02X - %s\n", token, e, (const char *)me);
    // Register it in the appropriate health tracker
    if (token == 0x1008) { 
      DHT0.healthTracker <<= 1; 
      DHT0.statusLED.start(DEVICE_ERROR_BLINK);
    } else if (token == 0x1009) { 
      DHT1.healthTracker <<= 1; 
      DHT1.statusLED.start(DEVICE_ERROR_BLINK);
    } else if (token == 0x2008 || token == 0x2009) { 
      targetHealth <<= 1; 
      targetLED.start(DEVICE_ERROR_BLINK);
    }
  } else {
    if (token == 0x1008 || token == 0x1009) { // Sensor data request
      // Get sensor slot
      mySensor& sensor = (token == 0x1008) ? DHT0 : DHT1;
      // get data
      uint16_t offs = 3;
      sensor.lockFlag = true;
      offs = response.get(offs, sensor.th.temperature);
      offs = response.get(offs, sensor.th.humidity);
      offs = response.get(offs, sensor.dewPoint);
      sensor.lockFlag = false;
      // Register successful request
      sensor.healthTracker <<= 1;
      sensor.healthTracker |= 1;
      sensor.statusLED.start(DEVICE_OK);
    } else if (token == 0x2008) { // target state request
      // Get data
      uint16_t stateT = 0;
      response.get(3, stateT);
      switchedON = (stateT > 0);
      // Register successful request
      targetHealth <<= 1;
      targetHealth |= 1;
      targetLED.start(DEVICE_OK);
    } else if (token == 0x2009) { // target switch request
      // Get data
      uint16_t stateT = 0;
      response.get(4, stateT);
      switchedON = (stateT > 0);
      // Register successful request
      targetHealth <<= 1;
      targetHealth |= 1;
      targetLED.start(DEVICE_OK);
    } else {
      // Unknown token?
      LOG_E("Unknown response %04X received.\n", token);
    }
  }
}

// Change target state to ON or OFF
void switchTarget(bool onOff) {
  LOG_V("Switch %s requested, switch is %s\n", onOff ? "ON" : "OFF", switchedON ? "ON" : "OFF");
  // We only need to do anything if the state is not the desired yet and we do have a target at all
  if (switchedON != onOff && settings.Target != DEV_NONE) {
    // State is different. Is it connected locally?
    if (settings.Target == DEV_LOCAL) {
      // Yes. Toggle target GPIO pin
      digitalWrite(TARGET_PIN, onOff);
    } else if (settings.Target == DEV_MODBUS) {
      MBclient.connect(settings.targetIP, settings.targetPort);
      Error e = MBclient.addRequest((uint32_t)0x2009, settings.targetSID, WRITE_HOLD_REGISTER, 1, onOff ? 1 : 0);
      if (e != SUCCESS) {
        ModbusError me(e);
        LOG_E("Error sending 0x2009 request: %02X - %s\n", e, (const char *)me);
      }
    }
    LOG_V("Switch request sent\n");
    // Register event
    registerEvent(onOff ? TARGET_ON : TARGET_OFF);
  }
  // Update signal LED anyway
  signalLED.start(onOff ? TARGET_ON_BLINK : TARGET_OFF_BLINK);
}

// Web server callbacks
// Illegal page requested
void notFound() {
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += HTMLserver.uri();
  message += "\nMethod: ";
  message += (HTMLserver.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += HTMLserver.args();
  message += "\n";
  for (uint8_t i = 0; i < HTMLserver.args(); i++) { message += " " + HTMLserver.argName(i) + ": " + HTMLserver.arg(i) + "\n"; }
  HTMLserver.send(404, "text/plain", message);
}

// Process config data received
void handleSet() {
  // Flag to note a change
  bool needsWrite = false;
  // Loop over all received args
  for (uint8_t i = 0; i < HTMLserver.args(); i++) { 
    // Get arg name as C-type pointer for atoi()
    const char *cp = HTMLserver.argName(i).c_str();
    // Is it a known config parameter?
    if (HTMLserver.argName(i).startsWith("CV")) {
      // Yes. Extract number
      uint8_t numbr = atoi(cp + 2);
      LOG_V("%3d: %s\n", numbr, HTMLserver.arg(i).c_str());
      
      // Try to extract numerical values
      cp = HTMLserver.arg(i).c_str();
      uint16_t uintval = atoi(cp);
      float floatval = atof(cp);
      // Determine sensor slot
      uint8_t sensor = 0;
      if (numbr >= 28 && numbr <= 41) {
        sensor = 1;
      }
      // IP byte index helper
      uint8_t ipIndex = 0;

      // Process depending on parameter
      switch (numbr) {
      case 0: // device name
        // Is it different from the existing value?
        if (strncmp(settings.deviceName, cp, STRINGPARMLENGTH)) {
          // Yes, we need to keep it
          strncpy(settings.deviceName, cp, STRINGPARMLENGTH - 1);
          needsWrite = true;
        }
        break;
      case 1: // WiFi SSID
        // Is it different from the existing value?
        if (strncmp(settings.WiFiSSID, cp, STRINGPARMLENGTH)) {
          // Yes, we need to keep it
          strncpy(settings.WiFiSSID, cp, STRINGPARMLENGTH - 1);
          needsWrite = true;
        }
        break;
      case 2: // WiFi password
        // Is it different from the existing value?
        if (strncmp(settings.WiFiPASS, cp, STRINGPARMLENGTH)) {
          // Yes, we need to keep it
          strncpy(settings.WiFiPASS, cp, STRINGPARMLENGTH - 1);
          needsWrite = true;
        }
        break;
      case 3: // OTA password
        // Is it different from the existing value?
        if (strncmp(settings.OTAPass, cp, STRINGPARMLENGTH)) {
          // Yes, we need to keep it
          strncpy(settings.OTAPass, cp, STRINGPARMLENGTH - 1);
          needsWrite = true;
        }
        break;
      case 4: // Master switch
        if ((uintval == 0) && settings.masterSwitch) { settings.masterSwitch = false; needsWrite = true; }
        if ((uintval != 0) && !settings.masterSwitch) { settings.masterSwitch = true; needsWrite = true; }
        break;
      case 5: // hysteresis steps
        // Only accept valid values
        if (uintval != settings.hystSteps 
          && uintval >= 1
          && uintval <= 16) { 
          settings.hystSteps = (uintval == 16) ? 0 : uintval; 
          needsWrite = true; 
        }
        break;
      case 6: // measuring interval
        // Only accept valid values
        if (uintval != settings.measuringInterval 
          && uintval >= 10
          && uintval <= 3600) { 
          settings.measuringInterval = uintval; 
          needsWrite = true; 
        }
        break;
      case 7: // target type
        if (uintval < 3 && uintval != settings.Target) {
          settings.Target = (DEVICEMODE)uintval;
          needsWrite = true;
        }
        break;
      case 8: // target Modbus IP
      case 9: 
      case 10: 
      case 11: 
        ipIndex = numbr - 8;
        if (uintval < 256 && settings.targetIP[ipIndex] != uintval) {
          settings.targetIP[ipIndex] = uintval;
          needsWrite = true;
        }
        break;
      case 12: // target Modbus port
        if (uintval >= 1 && uintval <= 65535 && uintval != settings.targetPort) {
          settings.targetPort = uintval;
          needsWrite = true;
        }
        break;
      case 13: // target Modbus SID
        if (uintval >= 1 && uintval <= 247 && uintval != settings.targetSID) {
          settings.targetSID = uintval;
          needsWrite = true;
        }
        break;
      case 14: // Sensor 0 type
      case 28: // Sensor 1 type
        if (uintval < 3 && uintval != settings.sensor[sensor].type) {
          settings.sensor[sensor].type = (DEVICEMODE)uintval;
          needsWrite = true;
        }
        break;
      case 15: // Sensor 0 Modbus IP
      case 29: // Sensor 1 Modbus IP
      case 16: //
      case 30: //
      case 17: //
      case 31: //
      case 18: //
      case 32: //
        ipIndex = numbr - 15 - sensor * 14;
        if (uintval < 256 && settings.sensor[sensor].IP[ipIndex] != uintval) {
          settings.sensor[sensor].IP[ipIndex] = uintval;
          needsWrite = true;
        }
        break;
      case 19: // Sensor 0 Modbus port
      case 33: // Sensor 1 Modbus port
        if (uintval >= 1 && uintval <= 65535 && uintval != settings.sensor[sensor].port) {
          settings.sensor[sensor].port = uintval;
          needsWrite = true;
        }
        break;
      case 20: // Sensor 0 Modbus SID
      case 34: // Sensor 1 Modbus SID
        if (uintval >= 1 && uintval <= 247 && uintval != settings.sensor[sensor].SID) {
          settings.sensor[sensor].SID = uintval;
          needsWrite = true;
        }
        break;
      case 21: // Sensor 0 Modbus slot
      case 35: // Sensor 1 Modbus slot
        if (uintval >= 1 && uintval <= 2 && uintval != (1 + (settings.sensor[sensor].slot ? 1 : 0))) {
          settings.sensor[sensor].slot = (uintval == 1 ? false : true);
          needsWrite = true;
        }
        break;
      case 22: // Sensor 0 temperature condition
      case 36: // Sensor 1 temperature condition
        if (uintval < 3 && uintval != settings.sensor[sensor].TempMode) {
          settings.sensor[sensor].TempMode = (DEVICECOND)uintval;
          needsWrite = true;
        }
        break;
      case 23: // Sensor 0 temperature condition value
      case 37: // Sensor 1 temperature condition value
        if (floatval != settings.sensor[sensor].Temp) {
          settings.sensor[sensor].Temp = floatval;
          needsWrite = true;
        }
        break;
      case 24: // Sensor 0 humidity condition
      case 38: // Sensor 1 humidity condition
        if (uintval < 3 && uintval != settings.sensor[sensor].HumMode) {
          settings.sensor[sensor].HumMode = (DEVICECOND)uintval;
          needsWrite = true;
        }
        break;
      case 25: // Sensor 0 humidity condition value
      case 39: // Sensor 1 humidity condition value
        if (floatval != settings.sensor[sensor].Hum) {
          settings.sensor[sensor].Hum = floatval;
          needsWrite = true;
        }
        break;
      case 26: // Sensor 0 dew point condition
      case 40: // Sensor 1 dew point condition
        if (uintval < 3 && uintval != settings.sensor[sensor].DewMode) {
          settings.sensor[sensor].DewMode = (DEVICECOND)uintval;
          needsWrite = true;
        }
        break;
      case 27: // Sensor 0 dew point condition value
      case 41: // Sensor 1 dew point condition value
        if (floatval != settings.sensor[sensor].Dew) {
          settings.sensor[sensor].Dew = floatval;
          needsWrite = true;
        }
        break;
      case 42: // Combo temperature condition
        if (uintval < 3 && uintval != settings.TempDiff) {
          settings.TempDiff = (DEVICECOND)uintval;
          needsWrite = true;
        }
        break;
      case 43: // Combo temperature condition value
        if (floatval != settings.Temp) {
          settings.Temp = floatval;
          needsWrite = true;
        }
        break;
      case 44: // Combo humidity condition
        if (uintval < 3 && uintval != settings.HumDiff) {
          settings.HumDiff = (DEVICECOND)uintval;
          needsWrite = true;
        }
        break;
      case 45: // Combo humidity condition value
        if (floatval != settings.Hum) {
          settings.Hum = floatval;
          needsWrite = true;
        }
        break;
      case 46: // Combo dew point condition
        if (uintval < 3 && uintval != settings.DewDiff) {
          settings.DewDiff = (DEVICECOND)uintval;
          needsWrite = true;
        }
        break;
      case 47: // Combo dew point condition value
        if (floatval != settings.Dew) {
          settings.Dew = floatval;
          needsWrite = true;
        }
        break;
      default: // Unhandled number
        LOG_I("CV parameter number unhandled [0..%d]: %d\n", CONFIGPARAMS, numbr);
        break;
      }
    } else {
      LOG_I("Unknown POST arg '%s'\n", cp);
    }
  }
  // Send result page
  String message(250);
  message = "<!DOCTYPE html><html><header>Done</header><body>";
  // If we found changes, report it.
  if (needsWrite) {
    writeSettings();
    message += "<br/>Configuration changes written.<br/>";
  }
  message += "<button onclick=\"window.location.href='/config.html';\"> CONFIG page </button>";
  message += "<button onclick=\"window.location.href='/restart';\"> Restart </button>";
  message += "</body></html>";
  HTMLserver.send(200, "text/html", message);
}

// Restart device
void handleRestart() {
  ESP.restart();
}

// Put out device status
void handleDevice() {
  String message(250);
  message = "<!DOCTYPE html><html><header>";
  if (*settings.deviceName) {
    message += settings.deviceName;
  } else {
    message += AP_SSID;
  }
  message += " status</header><body><hr/>";
  if (mode == CONFIG) {
    message += "<button onclick=\"window.location.href='/config.html';\"> CONFIG page </button>";
    message += "<button onclick=\"window.location.href='/restart';\"> Restart </button>";
  }
  message += "<br/>Restarts: ";
  message += settings.restarts;
  message += "<br/>";
  message += "<hr/></body></html>";
  HTMLserver.send(200, "text/html", message);
}

void setup() {
  // Assume target not active
  signalLED.start(TARGET_OFF_BLINK);

  // Init Serial monitor
  Serial.begin(115200);
  Serial.println("");
  Serial.println("__OK__");
  Serial.print("Version: ");
  Serial.println(VERSION);
  Serial.print("Build: ");
  Serial.println(BUILD_TIMESTAMP);

  // (Try to) init sensors
  DHT0.sensor.setup(SENSOR_0, DHTesp::DHT22);
  DHT1.sensor.setup(SENSOR_1, DHTesp::DHT22);

  // First check of sensors
  checkSensor(DHT0, "DHT0");
  checkSensor(DHT1, "DHT1");

  // Make time for held button detection longer - 1s
  tSwitch.setTiming(250, 1000);

  // Start file system handling
  LittleFS.begin();

  // Do we have a settings file?
  if (LittleFS.exists(SETTINGS)) {
    // Yes. Open it for read
    File sF = LittleFS.open(SETTINGS, "r");
    // Successfully opened?
    if (sF) {
      // Yes. Read in settings struct
      sF.readBytes((char *)&settings, sizeof(SetData));
      sF.close();
    } else {
      LOG_E("Settings file '" SETTINGS "' open failed.");
    }
  } else {
    LOG_E("Settings file '" SETTINGS "' does not exist.");
  }

  // Check if it is a valid settings file
  if (settings.magicValue == MAGICVALUE) {
    // It is - adjust runtime data with values from EEPROM
    // Increase boot count
    settings.restarts++;
    // Write back
    File sF = LittleFS.open(SETTINGS, "w");
    if (sF) {
      sF.write((char *)&settings, sizeof(SetData));
      sF.close();
    } else {
      LOG_E("Could not write '" SETTINGS "'");
    }
    // if we have neither WiFi access data nor a device name we need to go into CONFIG mode
    if (!*settings.deviceName || !*settings.WiFiPASS || !*settings.WiFiSSID) {
      mode = CONFIG;
    }
  } else {
    // No, fresh one, we need to initialize it
    settings.magicValue = MAGICVALUE;
    settings.restarts = 0;
    settings.masterSwitch = false;
    settings.hystSteps = 4;
    settings.measuringInterval = 20;
    settings.targetPort = 502;
    settings.targetSID = 1;
    settings.sensor[0].port = 502;
    settings.sensor[0].SID = 1;
    settings.sensor[0].slot = 1;
    settings.sensor[1].port = 502;
    settings.sensor[1].SID = 1;
    settings.sensor[1].slot = 1;

    int rc = writeSettings();
    if (rc != 0) {
      LOG_E("Writing initial settings failed (%d)\n", rc);
    }
    // Go into config mode in any case
    mode = CONFIG;
  }

  LOG_I("Restarts=%d\n", settings.restarts);

  // Create device name from flash ID, to be used as AP SSID in case
  strcpy(AP_SSID, "Dewair_XXXXXX");
  long id = ESP.getChipId();
  for (uint8_t i = 6; i; i--) {
    char c = (id & 0xf);
    if (c > 9) { c -= 10; c += 'A'; }
    else    c += '0';
    AP_SSID[6 + i] = c;
    id >>= 4;
  }

  // we will wait 3s for button presses to deliberately enter CONFIG mode
  signalLED.start(KNOB_BLINK, 100);
  uint32_t t0 = millis();

  while (millis() - t0 <= 3000) {
    signalLED.update();
    if (tSwitch.update() > 0) {
      // Button pressed?
      if (tSwitch.getEvent() != BE_NONE) {
        // YES. Force CONFIG mode and leave. 
        mode = CONFIG;
        break;
      }
    }
  }
  signalLED.stop();

  // Run mode?
  if (mode == RUN) {
    // Start WiFi client
    if (*settings.deviceName) {
      // Use given device name, if present
      wifiSetup(settings.deviceName);
    } else {
      // Use generated AP name instead
      wifiSetup(AP_SSID);
    }

    // Start NTP
    configTime(MY_TZ, MY_NTP_SERVER); 
    // Register boot event
    registerEvent(BOOT_DATE);
    registerEvent(BOOT_TIME);

    // Start mDNS
    MDNS.begin(settings.deviceName);

    // Calculate hysteresis mask
    uint8_t hw = settings.hystSteps == 0 ? 16 : settings.hystSteps;
    HYSTERESIS_MASK = (1 << hw) - 1;
    // Set interval
    INTERVAL_DHT = settings.measuringInterval * 1000;
    // Check boundaries
    if (INTERVAL_DHT < 10000) {
      INTERVAL_DHT = 10000;
    }
    if (INTERVAL_DHT > 3600000) {
      INTERVAL_DHT = 3600000;
    }
    // Register state of Master switch
    registerEvent(settings.masterSwitch ? MASTER_ON : MASTER_OFF);
    // Shorten idle timeout for Modbus client connections
    MBclient.setIdleTimeout(20000);
    // Register response handler
    MBclient.onResponseHandler(handleResponse);

    // Set up web server in RUN mode
    // need to exclude the config files in RUN mode!
    HTMLserver.on("/config.html", notFound);
    HTMLserver.on("/settings.bin", notFound);
    HTMLserver.on("/set.js", notFound);
    HTMLserver.on("/sub", notFound);
    HTMLserver.on("/restart", notFound);
    HTMLserver.on("/", handleDevice);

    // Set up Modbus server
    // *****************

    signalLED.start(TARGET_OFF_BLINK);
  } else {
    // No, config mode
    // Start AP.
    WiFi.softAP(AP_SSID, "Maelstrom");

    // Set up open web server in CONFIG mode
    HTMLserver.enableCORS(true);
    HTMLserver.on("/sub", handleSet);
    HTMLserver.on("/restart", handleRestart);
    HTMLserver.on("/", handleDevice);
    HTMLserver.serveStatic("/", LittleFS, "/");

    // Signal config mode
    signalLED.start(CONFIG_BLINK);
  }
  LOG_I("%s, mode=%d\n", AP_SSID, mode);

  // Set up mode independent web server callbacks
  HTMLserver.onNotFound(notFound);
  
  // Start web server
  HTMLserver.begin(80);
}

void loop() {
  // Timer for measurement cycle
  static uint32_t measure = millis();  
  // Timer for runtime counting
  static uint32_t tick = millis();
  // Results of last conditions check
  static uint8_t s1cond = 0;
  static uint8_t s2cond = 0;
  static uint8_t cccond = 0;

  // Keep track of blinking status LEDs and button presses
  signalLED.update();
  targetLED.update();
  S1LED.update();
  S2LED.update();

  // Update web server
  HTMLserver.handleClient();

  // Update mDNS
  MDNS.update();

  // RUN mode?
  if (mode == RUN) {
    if (tSwitch.update() > 0) {
      // Button pressed?
      ButtonEvent be = tSwitch.getEvent();
      // Short press?
      if (be == BE_CLICK) {
        // Yes, single click. Check sensors again
        checkSensor(DHT0, "DHT0");
        checkSensor(DHT1, "DHT1");
      } else if (be == BE_PRESS) {
        // No, button was held. Switch to manual mode
        signalLED.start(MANUAL_BLINK);
        S1LED.stop();
        S2LED.stop();
        targetLED.stop();
        mode = MANUAL;
      } else if (be == BE_DOUBLECLICK) {
        // No again, we caught a double click instead
        // Let the three device LEDs signal the switch logic calculations result
        // Will be normalized as soon as the next measuring cycle is begun.
        // Sensor 0 all conditions met?
        if (s1cond == 3) { S1LED.start(MANUAL_BLINK);  // Yes, LED steady on
        } else           { S1LED.stop(); }             // No, LED off
        // Sensor 1 all conditions met?
        if (s2cond == 3) { S2LED.start(MANUAL_BLINK);
        } else           { S2LED.stop(); }
        // Combined conditions all met?
        if (cccond == 3) { targetLED.start(MANUAL_BLINK);
        } else           { targetLED.stop(); }
      }
    }
  
    // Is it time to advance the runtime counter?
    // (do a target state check as well)
    if (millis() - tick >= 60000) {
      // Yes, increment it
      runTime++;
      // Check for data change
      time_t now = time(NULL);
      tm tm;
      localtime_r(&now, &tm);           // update the structure tm with the current time
      if (tm.tm_hour == 0 && tm.tm_min == 0) {
        registerEvent(DATE_CHANGE);
      }
      // Is a target configured?
      if (settings.Target != DEV_NONE) {
        // Yes. get switch state
        // Is it a Modbus device?
        if (settings.Target == DEV_MODBUS) {
          // Yes, send a request
          MBclient.connect(settings.targetIP, settings.targetPort);
          Error e = MBclient.addRequest((uint32_t)0x2008, settings.targetSID, READ_HOLD_REGISTER, 1, 1);
          if (e != SUCCESS) {
            ModbusError me(e);
            Serial.printf("Error sending request 0x2008: %02X - %s\n", e, (const char *)me);
          }
          LOG_V("Switch status requested\n");
        } else {
          // No, shall be local
          // We must trust on the wiring, so we assume good health
          switchedON = digitalRead(TARGET_PIN);
          targetHealth <<= 1;
          targetHealth |= 1;
          targetLED.start(DEVICE_OK);
        }
      }
      // Debug output
      LOG_V("Health tracker: S1=%04X S2=%04X Tg=%04X\n", DHT0.healthTracker, DHT1.healthTracker, targetHealth);
      // Reset timer
      tick = millis();
    }

    // Is it time to do another measurement?
    if (millis() - measure > INTERVAL_DHT) {
      //  Check switch conditions
      s1cond = 0;
      s2cond = 0;
      cccond = 0;
      // Yes. Check sensor 0
      if (takeMeasurement(DHT0, "DHT0")) {
        // Fine, check sensor 1
        if (takeMeasurement(DHT1, "DHT1")) {
          // All data gathered. Kill oldest hysteresis bit
          Hysteresis <<= 1;
          // Check both sensors
          for (uint8_t i = 0; i < 2; i++) {
            uint8_t& checks = (i == 0 ? s1cond : s2cond);
            // Sensor not connected? Counts as all conditions met
            if (settings.sensor[i].type == DEV_NONE) {
                checks = 3;
            } else {
              // No, we have a sensor 
              mySensor& sensor = (i == 0) ? DHT0 : DHT1;
              // Wait while data is being written (handleResponse())
              while (sensor.lockFlag) { delay(10); }
              // 1: Check temperature
              switch (settings.sensor[i].TempMode) {
              case 0:  checks++; break;
              case 1:  if (sensor.th.temperature < settings.sensor[i].Temp) { checks++; } break;
              case 2:  if (sensor.th.temperature > settings.sensor[i].Temp) { checks++; } break;
              }
              // 2: Check humidity
              switch (settings.sensor[i].HumMode) {
              case 0:  checks++; break;
              case 1:  if (sensor.th.humidity < settings.sensor[i].Hum) { checks++; } break;
              case 2:  if (sensor.th.humidity > settings.sensor[i].Hum) { checks++; } break;
              }
              // 3: Check dew point
              switch (settings.sensor[i].DewMode) {
              case 0:  checks++; break;
              case 1:  if (sensor.dewPoint < settings.sensor[i].Dew) { checks++; } break;
              case 2:  if (sensor.dewPoint > settings.sensor[i].Dew) { checks++; } break;
              }
            }
          }
          // Finally check combo conditions
          // If we have one sensor only, count as all conditions met
          if (settings.sensor[0].type == DEV_NONE || settings.sensor[1].type == DEV_NONE) {
            cccond += 3;
          } else {
            // We have both sensors.
            // Check temperature
            switch (settings.TempDiff) {
            case 0: cccond++; break;
            case 1: if (DHT0.th.temperature - DHT1.th.temperature < settings.Temp) { cccond++; } break;
            case 2: if (DHT0.th.temperature - DHT1.th.temperature > settings.Temp) { cccond++; } break;
            }
            // Check humidity
            switch (settings.HumDiff) {
            case 0: cccond++; break;
            case 1: if (DHT0.th.humidity - DHT1.th.humidity < settings.Hum) { cccond++; } break;
            case 2: if (DHT0.th.humidity - DHT1.th.humidity > settings.Hum) { cccond++; } break;
            }
            // Check dew point
            switch (settings.DewDiff) {
            case 0: cccond++; break;
            case 1: if (DHT0.dewPoint - DHT1.dewPoint < settings.Dew) { cccond++; } break;
            case 2: if (DHT0.dewPoint - DHT1.dewPoint > settings.Dew) { cccond++; } break;
            }
          }
          // All conditions met?
          if (s1cond + s2cond + cccond == 9) {
            Hysteresis |= 1;
          }
          // Shall we be active?
          if (settings.masterSwitch) {
            // Yes, Determine resulting switch state
            bool desiredStateON = false;
            // Did all considered measurements suggest ON state?
            if ((Hysteresis & HYSTERESIS_MASK) == HYSTERESIS_MASK) {
              // Yes, note it.
              desiredStateON = true;
            }
            // Switch target (if necessary)
            switchTarget(desiredStateON);
          }
        }
      }
      // Debug output
      LOG_V("S1 %5.1f %5.1f %5.1f\n", DHT0.th.temperature, DHT0.th.humidity, DHT0.dewPoint);
      LOG_V("S2 %5.1f %5.1f %5.1f\n", DHT1.th.temperature, DHT1.th.humidity, DHT1.dewPoint);
      LOG_V("    Check=%d/%d/%d\n", s1cond, s2cond, cccond);
      measure = millis();
    }
  } else if (mode == MANUAL) {
    // We are in manual mode.
    // Button pressed?
    if (tSwitch.update() > 0) {
      // Yes, get it
      ButtonEvent be = tSwitch.getEvent();
      // Short press?
      if (be == BE_CLICK) {
        // Yes, single click. toggle target
        switchTarget(!switchedON);
      } else if (be == BE_PRESS) {
        // No, button was held. Switch to run mode
        signalLED.start(switchedON ? TARGET_ON_BLINK : TARGET_OFF_BLINK);
        checkSensor(DHT0, "DHT0");
        checkSensor(DHT1, "DHT1");
        mode = RUN;
      }
    }
  } else {
    // No, CONFIG mode!
    // **********************
  }
}
