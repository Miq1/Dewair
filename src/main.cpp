// Copyright 2022 Michael Harwerth miq1 _ at _ gmx _ dot _ de

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESPAsyncTCP.h>
#include <ESP8266mDNS.h>
#include <LittleFS.h>
#include <ESP8266WebServer.h>
#include "DHTesp.h"
#include "Version.h"
#include "Blinker.h"
#include "Buttoner.h"
#include "RingBuf.h"
#include "ModbusClientTCPAsync.h"
#include "ModbusServerTCPAsync.h"
#include "Logging.h"

// Pin definitions
#define SENSOR_0 D2
#define SENSOR_1 D1
#define SIGNAL_LED D6
#define S0STATUS_LED D4
#define S1STATUS_LED D5
#define TARGET_LED D0
#define SWITCH_PIN D3
#define TARGET_PIN D8

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
#define SETTINGS "/settings.bin"
#define RESTARTS "/restarts.bin"
String deviceInfo(1024);

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
uint16_t cState = 0;                                        // last evaluation result for switching conditions

// Target tracking
uint16_t targetHealth = 0;
bool switchedON = false;                                    // Current target switch state
// Modbus Error tracking
struct TT {
  Modbus::Error err;                                        // Error proper
  uint16_t count;                                           // Number of consecutive occurrences
  TT() : err(SUCCESS), count(0) {}
};
const uint16_t TTslots(30);                                 // Number of error groups tracked
uint16_t ttSlot{0};                                         // Currently active group
TT targetTrack[TTslots];                                    // Storage for error tracking

// Set up blink patterns for all 4 LEDs
Blinker signalLED(SIGNAL_LED);
Blinker S0LED(S0STATUS_LED);
Blinker S1LED(S1STATUS_LED);
Blinker targetLED(TARGET_LED);

// Blink pattern for "target ON": quick triple, pause
const uint16_t TARGET_ON_BLINK(0xA800);
// Blink pattern for "target OF": single, pause
const uint16_t TARGET_OFF_BLINK(0x1000);
// Wait for initial button press: quick on/off sequence, no pause
const uint16_t KNOB_BLINK(0x1111);
// CONFIG mode: long triple, pause
const uint16_t CONFIG_BLINK(0xCCC0);
// Wifi connect mode: long on/off, no pause
const uint16_t WIFI_BLINK(0xFF00);
// Manual mode: steady on
const uint16_t MANUAL_BLINK(0xFFFF);
// Device ignored: steady off
const uint16_t DEVICE_IGNORED(0x0000);
// Device OK: steady on
const uint16_t DEVICE_OK(0xFFFF);
// Device error: even quicker on/off sequence, no pause
const uint16_t DEVICE_ERROR_BLINK(0xAAAA);

// Switch to trigger restarts etc.
Buttoner tSwitch(SWITCH_PIN, LOW);

// Flag for remote reboot request
uint8_t rebootPending = 0;                // if != 0, reboot is awaiting second request
long unsigned int rebootGrace = 0;        // Wait timer for confirming second reboot request

// Two DHT sensors. In setup() will be found out if both are connected
struct mySensor {
  DHTesp sensor;                               // DHT object
  TempAndHumidity th;                          // Measured values
  float dewPoint;                              // Calculated from measurement
  Blinker& statusLED;
  uint16_t healthTracker;
  uint8_t sensor01;                            // Slot 0 or 1
  bool isRelevant;
  bool lastCheckOK;
  mySensor(Blinker& sLED, uint8_t whichOne) : 
    statusLED(sLED), 
    healthTracker(0), 
    sensor01(whichOne), 
    isRelevant(false),
    lastCheckOK(false) { }
};
mySensor DHT0(S0LED, 0);
mySensor DHT1(S1LED, 1);

// Choice list for target and sensor devices
enum DEVICEMODE : uint8_t { DEV_NONE=0, DEV_LOCAL, DEV_MODBUS, DEV_RESERVED };
// Choice list for conditions
enum DEVICECOND : uint8_t { DEVC_NONE=0, DEVC_LESS, DEVC_GREATER, DEVC_RESERVED };
// Defined class for IP port numbers to do proper error checking
class PORTNUM {
protected:
  uint16_t value;
public:
  PORTNUM() : value(0) {}
  explicit PORTNUM(uint16_t v) : value(v) {}
  operator uint16_t() { return value; }
  uint16_t operator=(uint16_t v) { return (value = v); }
};
// Defined class for Modbus server ID to do proper error checking
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
const uint16_t MAGICVALUE(0x4716);
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
  bool fallbackSwitch;                   // CV48 Fallback if sensors etc. will fail
  SetData() {
    magicValue = 0;
  }
} settings;
uint16_t restarts;                     // number of reboots

// History data 
// Measurements are stored for 24h
const uint16_t HistorySlots(120);      // Number of measurements collected in 24h
const uint16_t HistoryAddress(400);    // Modbus register number of first history data
struct HistoryEntry {
  uint16_t temp0;                      // Sensor 0 temperature t0 as: uint16_t((t0 + 100.0) * 10.0) 
  uint16_t hum0;                       // Sensor 0 humidity h0 as: uint16_t(h0 * 10.0) 
  uint16_t temp1;                      // Sensor 1 temperature t1 as: uint16_t((t1 + 100.0) * 10.0) 
  uint16_t hum1;                       // Sensor 1 humidity h1 as: uint16_t(h1 * 10.0) 
  uint8_t on;                          // Target ON state as: (samples(ON) * 100) / samples
};
// Storage for history data
HistoryEntry history[HistorySlots];
// History evaluation struct
class CalcHistory {
protected:
  float t0sum;                         // Sum of all t0 values in a sampling period
  float h0sum;                         // Sum of all h0 values in a sampling period
  float t1sum;                         // Sum of all t0 values in a sampling period
  float h1sum;                         // Sum of all h0 values in a sampling period
  uint16_t onCnt;                      // Number of samples with target==ON
  uint16_t count;                      // Number of samples collected
  uint16_t historySlot;         // Current slot
  // reset: init counters
  void reset() {
    t0sum = h0sum = t1sum = h1sum = 0.0;
    onCnt = count = 0;
  }
  // push: move collected data into history slot
  void push(HistoryEntry& hE) {
    if (count) {
      hE.temp0 = (uint16_t)(((t0sum / count) + 100.0) * 10.0);
      hE.hum0 = (uint16_t)((h0sum / count) * 10.0);
      hE.temp1 = (uint16_t)(((t1sum / count) + 100.0) * 10.0);
      hE.hum1 = (uint16_t)((h1sum / count) * 10.0);
      hE.on = (uint8_t)((onCnt * 100) / count);
    }
  }
public:
  // Constructor
  CalcHistory() : historySlot(0) { reset(); }
  // calcSlot: determine history slot from hour and minute of current time
  static uint16_t calcSlot() {
    // We will need date and/or time
    time_t now = time(NULL);
    tm tm;
    localtime_r(&now, &tm);           // update the structure tm with the current time
    uint16_t minV = tm.tm_hour * 60 + tm.tm_min;    // Minute of day
    return HistorySlots ? minV / (1440 / HistorySlots) : 0;   // find slot
  }
  // collect: add another set of values
  uint16_t collect(float t0, float h0, float t1, float h1, bool on) {
    // get current slot
    uint16_t actSlot = calcSlot();
    // Has it changed?
    if (actSlot != historySlot) {
      // Yes. we need to move the collected data into history and adwance to the next
      push(history[historySlot]);
      historySlot = actSlot;
      reset();
    }
    count++;
    // If value is NaN or humidity is zero (unlikely...), do not use it, but promote the current average
    if (isnanf(t0) || h0 == 0.0) {
      t0sum += t0sum / count;
    } else {
      t0sum += t0;
    }
    // If value is NaN or humidity is zero (unlikely...), do not use it, but promote the current average
    if (isnanf(h0) || h0 == 0.0) {
      h0sum += h0sum / count;
    } else {
      h0sum += h0;
    }
    // If value is NaN or humidity is zero (unlikely...), do not use it, but promote the current average
    if (isnanf(t1) || h1 == 0.0) {
      t1sum += t1sum / count;
    } else {
      t1sum += t0;
    }
    // If value is NaN or humidity is zero (unlikely...), do not use it, but promote the current average
    if (isnanf(h1) || h1 == 0.0) {
      h1sum += h1sum / count;
    } else {
      h1sum += h0;
    }
    onCnt += (on ? 1 : 0);
    return count;
  }
};
CalcHistory calcHistory;

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
      writeSetting(sJ, head, 48, (uint8_t)(settings.fallbackSwitch ? 1 : 0));
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

// Keep track of Modbus error responses
void registerMBerror(Modbus::Error e) {
  // Only sensible if we have slots at all
  if (TTslots) {
    // Is the code the same as before?
    if (e == targetTrack[ttSlot].err) {
      // Yes. Did we count less than the counter can hold?
      if (targetTrack[ttSlot].count < 65535) {
        // Yes. Increase counter
        targetTrack[ttSlot].count++;
      }
    } else {
      // No it is different. Advanvce index
      ttSlot++;
      // did we reach the end of slots?
      if (ttSlot >= TTslots) {
        // Yes, wrap around
        ttSlot = 0;
      }
      // Write error and init count
      targetTrack[ttSlot].err = e;
      targetTrack[ttSlot].count = 1;
    }
  }
}

// Number of event slots
const uint8_t MAXEVENT(40);
// Define the event types
enum S_EVENT : uint8_t  { 
  NO_EVENT=0, DATE_CHANGE,
  BOOT_DATE, BOOT_TIME, 
  MASTER_ON, MASTER_OFF,
  TARGET_ON, TARGET_OFF,
  ENTER_MAN, EXIT_MAN,
  FAIL_FB,
};
const char *eventname[] = { 
  "no event", "date change", "boot date", "boot time", 
  "MASTER on", "MASTER off", 
  "target on", "target off", 
  "enter manual", "exit manual",
  "failure fallback",
};
// Allocate event buffer
RingBuf<uint16_t> events(MAXEVENT);

// Server for own data
ModbusServerTCPasync MBserver;
// Modbus server ID
const uint8_t MYSID(1);
// Client to access switch and sensor sources
ModbusClientTCPasync MBclient(10);

// checkSensor: test if physical sensor is functional
int checkSensor(mySensor& ms) {
  int rc = -1;

  // advance health tracker
  ms.healthTracker <<= 1;
  // The DHT lib will return nan values if no sensor is present
  if (!isnan(ms.sensor.getTemperature())) {
    // We got a value - sensor seems to be functional
    LOG_I("Sensor %u ok.\n", ms.sensor01);
    ms.healthTracker |= 1;
    // Light upper status LED
    ms.statusLED.start(DEVICE_OK);
    ms.lastCheckOK = true;
    rc = 0;
  } else {
    // We caught a nan.
    LOG_E("Sensor %u: error %s\n", ms.sensor01, ms.sensor.getStatusString());
    // Turn off status LED
    ms.statusLED.start(DEVICE_IGNORED);
    ms.lastCheckOK = false;
  }
  return rc;
}

// takeMeasurement: get temperature, humidity and dew point for a sensor
// If no sensor is connected, try to get data from Modbus source
bool takeMeasurement(mySensor& ms) {
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
      ms.lastCheckOK = true;
    } else {
      ms.statusLED.start(DEVICE_ERROR_BLINK);
      ms.lastCheckOK = false;
    }
  // No, but a Modbus source perhaps?
  } else if (sd.type == DEV_MODBUS) {
    // Yes. Check Modbus
    // Is Modbus address set?
    if (sd.IP && sd.port && sd.SID) {
      // Yes. Set target and send a request
      // This will be asynchronous, so we may use the previous data for now.
      // MBclient.connect(sd.IP, sd.port);
      MBclient.setTarget(sd.IP, sd.port);
      Error e = MBclient.addRequest((uint32_t)((millis() << 16) | (0x1008 | ms.sensor01)), 
        sd.SID, 
        READ_HOLD_REGISTER,
        (uint16_t)(sd.slot ? 8 : 2),  // Register address slot 1:2, slot 2:2 + 6 = 8
        (uint16_t)6);
      if (e != SUCCESS) {
        ModbusError me(e);
        LOG_E("Error requesting sensor %d - %s\n", ms.sensor01, (const char *)me);
        registerMBerror(e);
        ms.lastCheckOK = false;
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

// writeDeviceInfo: update Web page data on device settings
void writeDeviceInfo() {
  const uint8_t BUFLEN(200);
  char buf[200];
  // HTML lead-in
  deviceInfo = "<!DOCTYPE html> <html><header> <link rel=\"stylesheet\" href=\"styles.css\"> </header> <body><hr/>\n";
  snprintf(buf, BUFLEN, "<h2>%s status</h2>\n", *settings.deviceName ? settings.deviceName : AP_SSID);
  deviceInfo += buf;
  // SW version etc
  deviceInfo += "<table>\n";
  deviceInfo += "<tr align=\"left\"><th>Version</th><td>" VERSION "</td></tr>\n";
  deviceInfo += "<tr align=\"left\"><th>Build</th><td>" BUILD_TIMESTAMP "</td></tr>\n";
  snprintf(buf, BUFLEN, "<tr align=\"left\"><th>Restarts</th><td>%d</td>\n", restarts);
  deviceInfo += buf;
  // Master switch state
  snprintf(buf, BUFLEN, "<tr align=\"left\"><th>Master switch</th><td>%s</td>\n", settings.masterSwitch ? "ON" : "OFF");
  deviceInfo += buf;
  // Fallback switch state
  snprintf(buf, BUFLEN, "<tr align=\"left\"><th>Fallback: switch to</th><td>%s</td>\n", settings.fallbackSwitch ? "ON" : "OFF");
  deviceInfo += buf;
  // Hysteresis settings
  snprintf(buf, BUFLEN, "<tr align=\"left\"><th>Measuring every</th><td>%d seconds</td>\n", settings.measuringInterval);
  deviceInfo += buf;
  // Target
  deviceInfo += "<tr align=\"left\"><th>Target</th><td>";
  switch (settings.Target) { 
  case DEV_NONE:
    deviceInfo += "none"; 
    break;
  case DEV_LOCAL:
    deviceInfo += "connected locally"; 
    break;
  case DEV_MODBUS:
    deviceInfo += "Modbus TCP"; 
    break;
  case DEV_RESERVED:
    deviceInfo += "reserved"; 
    break;
  }
  deviceInfo += "</td></tr>\n";
  // Sensors
  for (uint8_t i = 0; i < 2; i++) {
    deviceInfo += "<tr align=\"left\"><th>Sensor ";
    deviceInfo += i;
    deviceInfo += "</th><td>";
    switch (settings.sensor[i].type) { 
    case DEV_NONE:
      deviceInfo += "none"; 
      break;
    case DEV_LOCAL:
      deviceInfo += "connected locally"; 
      break;
    case DEV_MODBUS:
      deviceInfo += "Modbus TCP"; 
      break;
    case DEV_RESERVED:
      deviceInfo += "reserved"; 
      break;
    }
    deviceInfo += "</td></tr>\n";
  }
  // Conditions for switching
  snprintf(buf, BUFLEN, "<tr align=\"left\"><th>Switching on</th><td>%d consecutive identical evaluations</td>\n", settings.hystSteps);
  deviceInfo += buf;
  deviceInfo += "<tr align=\"left\"><th>Switch conditions</th><td>";
  const char *leadIn = "IF ";
  for (uint8_t i = 0; i < 2; i++) {
    if (settings.sensor[i].type) {
      if (settings.sensor[i].TempMode) {
        snprintf(buf, BUFLEN, "%s S%d temperature %s %5.1f<br/>", leadIn, i, settings.sensor[i].TempMode == 1 ? "below" : "above", settings.sensor[i].Temp);
        deviceInfo += buf;
        leadIn = "AND ";
      }
      if (settings.sensor[i].HumMode) {
        snprintf(buf, BUFLEN, "%s S%d humidity %s %5.1f<br/>", leadIn, i, settings.sensor[i].HumMode == 1 ? "below" : "above", settings.sensor[i].Hum);
        deviceInfo += buf;
        leadIn = "AND ";
      }
      if (settings.sensor[i].DewMode) {
        snprintf(buf, BUFLEN, "%s S%d temperature %s %5.1f<br/>", leadIn, i, settings.sensor[i].DewMode == 1 ? "below" : "above", settings.sensor[i].Dew);
        deviceInfo += buf;
        leadIn = "AND ";
      }
    }
  }
  if (settings.sensor[0].type && settings.sensor[1].type) {
    if (settings.TempDiff) {
      snprintf(buf, BUFLEN, "%s (S0 temperature - S1 temperature) %s %5.1f<br/>", leadIn, settings.TempDiff == 1 ? "below" : "above", settings.Temp);
      deviceInfo += buf;
      leadIn = "AND ";
    }
    if (settings.HumDiff) {
      snprintf(buf, BUFLEN, "%s (S0 humidity - S1 humidity) %s %5.1f<br/>", leadIn, settings.HumDiff == 1 ? "below" : "above", settings.Hum);
      deviceInfo += buf;
      leadIn = "AND ";
    }
    if (settings.DewDiff) {
      snprintf(buf, BUFLEN, "%s (S0 dew point - S1 dew point) %s %5.1f<br/>", leadIn, settings.DewDiff == 1 ? "below" : "above", settings.Dew);
      deviceInfo += buf;
      leadIn = "AND ";
    }
  }
  if (!strcmp(leadIn, "IF ")) {
    deviceInfo += "no restriction<br/>\n";
  }
  deviceInfo += "</td></tr>\n";
  deviceInfo += "</table>\n";
  // HTML trailer
  deviceInfo += "<hr/></body></html>\n";
  LOG_V("deviceInfo=%d\n", deviceInfo.length());
}

// Helper function to pack some Modbus register values
uint16_t makeCompact(uint8_t type, uint16_t value) {
  return ((type & 0x03)  << 14) | (value & 0x0FFF);
}

// Modbus server READ_HOLD_REGISTER callback
ModbusMessage FC03(ModbusMessage request) {
  ModbusMessage response;          // returned response message

  uint16_t address = 0;
  uint16_t words = 0;

  // Get start address and length for read
  request.get(2, address);
  request.get(4, words);

  // Valid address etc.?
  if (address && words && address + words <= 65 + MAXEVENT + 1 + TTslots * 2) {
    // Yes, looks good. Prepare response header
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
    // Temporary buffer for uint16_t manipulations
    uint16_t uVal;
    // We may need the float measurement values, so prepare a Modbus-compliant format
    ModbusMessage fVal;
    if (address <= 13 && 2 <= address + words - 1) {
      fVal.add(DHT0.th.temperature);
      fVal.add(DHT0.th.humidity);
      fVal.add(DHT0.dewPoint);
      fVal.add(DHT1.th.temperature);
      fVal.add(DHT1.th.humidity);
      fVal.add(DHT1.dewPoint);
    }

    // Loop over all requested addresses
    for (uint16_t a = address; a < address + words; a++) {
      switch (a) {
      case  1: // Master switch
        uVal = settings.masterSwitch ? 1 : 0;
        response.add(uVal);
        break;
      case  2: // S0 temperature upper word
      case  3: // S0 temperature lower word
      case  4: // S0 humidity upper word
      case  5: // S0 humidity lower word
      case  6: // S0 dew point upper word
      case  7: // S0 dew point lower word
      case  8: // S1 temperature upper word
      case  9: // S1 temperature lower word
      case 10: // S1 humidity upper word
      case 11: // S1 humidity lower word
      case 12: // S1 dew point upper word
      case 13: // S1 dew point lower word
        // Get word from prepared buffer
        fVal.get((a - 2) * 2, uVal);
        response.add(uVal);
        break;
      case 14: // Target switch state
        uVal = switchedON ? 1 : 0;
        response.add(uVal);
        break;
      case 15: // restart count
        response.add(restarts);
        break;
      case 16: // run time since boot
        response.add(runTime);
        break;
      case 17: // S0 health
        response.add(DHT0.healthTracker);
        break;
      case 18: // S1 health
        response.add(DHT1.healthTracker);
        break;
      case 19: // target health
        response.add(targetHealth);
        break;
      case 20: // measurement interval
        response.add(settings.measuringInterval);
        break;
      case 21: // hysteresis steps
        response.add((uint16_t)settings.hystSteps);
        break;
      case 22: // S0 type
        response.add((uint16_t)settings.sensor[0].type);
        break;
      case 23: // S0 Modbus IP bytes 0, 1
        uVal = (settings.sensor[0].IP[0] << 8) | settings.sensor[0].IP[1];
        response.add(uVal);
        break;
      case 24: // S0 Modbus IP bytes 2, 3
        uVal = (settings.sensor[0].IP[2] << 8) | settings.sensor[0].IP[3];
        response.add(uVal);
        break;
      case 25: // S0 Modbus port
        response.add(settings.sensor[0].port);
        break;
      case 26: // S0 Modbus SID and slot
        uVal = (settings.sensor[0].SID << 8) | settings.sensor[0].slot;
        response.add(uVal);
        break;
      case 27: // S0 temperature condition and value
        uVal = int(settings.sensor[0].Temp * 10) + 2048;
        response.add(makeCompact(settings.sensor[0].TempMode, uVal));
        break;
      case 28: // S0 humidity condition and value
        uVal = int(settings.sensor[0].Hum * 10) + 2048;
        response.add(makeCompact(settings.sensor[0].HumMode, uVal));
        break;
      case 29: // S0 dew point condition and value
        uVal = int(settings.sensor[0].Dew * 10) + 2048;
        response.add(makeCompact(settings.sensor[0].DewMode, uVal));
        break;
      case 30: // S1 type
        response.add((uint16_t)settings.sensor[1].type);
        break;
      case 31: // S1 Modbus IP bytes 0, 1
        uVal = (settings.sensor[1].IP[0] << 8) | settings.sensor[1].IP[1];
        response.add(uVal);
        break;
      case 32: // S1 Modbus IP bytes 2, 3
        uVal = (settings.sensor[1].IP[2] << 8) | settings.sensor[1].IP[3];
        response.add(uVal);
        break;
      case 33: // S1 Modbus port
        response.add(settings.sensor[1].port);
        break;
      case 34: // S1 Modbus SID and slot
        uVal = (settings.sensor[1].SID << 8) | settings.sensor[1].slot;
        response.add(uVal);
        break;
      case 35: // S1 temperature condition and value
        uVal = int(settings.sensor[1].Temp * 10) + 2048;
        response.add(makeCompact(settings.sensor[1].TempMode, uVal));
        break;
      case 36: // S1 humidity condition and value
        uVal = int(settings.sensor[1].Hum * 10) + 2048;
        response.add(makeCompact(settings.sensor[1].HumMode, uVal));
        break;
      case 37: // S1 dew point condition and value
        uVal = int(settings.sensor[1].Dew * 10) + 2048;
        response.add(makeCompact(settings.sensor[1].DewMode, uVal));
        break;
      case 38: // Target type
        uVal = settings.Target;
        response.add(uVal);
        break;
      case 39: // target Modbus IP bytes 0, 1
        uVal = (settings.targetIP[0] << 8) | settings.targetIP[1];
        response.add(uVal);
        break;
      case 40: // target Modbus IP bytes 2, 3
        uVal = (settings.targetIP[2] << 8) | settings.targetIP[3];
        response.add(uVal);
        break;
      case 41: // target Modbus port
        response.add(settings.targetPort);
        break;
      case 42: // target Modbus SID
        uVal = settings.targetSID << 8;
        response.add(uVal);
        break;
      case 43: // combo temperature condition type and value
        uVal = int(settings.Temp * 10) + 2048;
        response.add(makeCompact(settings.TempDiff, uVal));
        break;
      case 44: // combo humidity condition type and value
        uVal = int(settings.Hum * 10) + 2048;
        response.add(makeCompact(settings.HumDiff, uVal));
        break;
      case 45: // combo dew point condition type and value
        uVal = int(settings.Dew * 10) + 2048;
        response.add(makeCompact(settings.DewDiff, uVal));
        break;
      case 46: // current condition state
        response.add(cState);
        break;
      case 47: // Fallback switch
        uVal = settings.fallbackSwitch ? 1 : 0;
        response.add(uVal);
        break;
      case 48: // history slot count
        response.add((uint16_t)HistorySlots);
        break;
      case 49: // history start register address
        response.add((uint16_t)HistoryAddress);
        break;
      // reserved register numbers left out
      case 64: // event slot count
        response.add((uint16_t)MAXEVENT);
        break;
      case 65 ... (65 + MAXEVENT - 1): // Events
        response.add(events[a - 65]);
        break;
      case 65 + MAXEVENT: // Error tracking slots
        response.add(TTslots);
        break;
      case 66 + MAXEVENT ... 65 + MAXEVENT + TTslots * 2: // Error tracking values
        {
          // Calculate index. Subtract start address and divide by two.
          uint16_t relIndex = (a - (66 + MAXEVENT)) / 2;
          // Now calculate slot from it. We may have to go back and around!
          uint16_t slot = (ttSlot + TTslots - relIndex) % TTslots;
          // Odd index (=count)?
          if ((a - (66 + MAXEVENT)) & 1) {
            // Yes. Return count
            response.add(targetTrack[slot].count);
          } else {
            // No, error code requested
            response.add((uint16_t)targetTrack[slot].err);
          }
        }
        break;
      default: // Reserve registers
        response.add((uint16_t)0);
        break;
      }
    }
  // None of the regular registers, but is it in the history area?
  } else if (HistorySlots && words && address >= HistoryAddress && (address + words) <= (HistoryAddress + 5 * HistorySlots)) {
    // Yes, looks good. Prepare response header
    response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));
    // Loop over all requested addresses
    for (uint16_t a = address; a < address + words; a++) {
      // Determine data type requested:
      //   0: sensor 0 temperatures
      //   1: sensor 0 humidities
      //   2: sensor 1 temperatures
      //   3: sensor 1 humidities
      //   4: target on ratio
      uint8_t type = (a - HistoryAddress) / HistorySlots;
      // Calculate index in history
      uint16_t offs = (a - HistoryAddress) % HistorySlots;
      switch (type) {
      case 0: // temp0
        response.add((uint16_t)history[offs].temp0);
        break;
      case 1: // hum0
        response.add((uint16_t)history[offs].hum0);
        break;
      case 2: // temp1
        response.add((uint16_t)history[offs].temp1);
        break;
      case 3: // hum1
        response.add((uint16_t)history[offs].hum1);
        break;
      case 4: // target ON
        response.add((uint16_t)history[offs].on);
        break;
      default: // why???
        LOG_E("Unknown history type %d?\n", type);
        break;
      }
    }
  } else {
    // No, addressable registers were missed in a way. Return error message
    response.setError(request.getServerID(), request.getFunctionCode(), ILLEGAL_DATA_ADDRESS);
    LOG_V("Address error: addr=%d words=%d\n", address, words);
  }
  return response;
}

// writeRegister: helper function to check a register address and data
//    if it can be written. Write it, if permissible
Error writeRegister(uint16_t address, uint16_t value) {
  Error rc = SUCCESS;              // Function return value

  // Check address range
  if (address && address <= 65 + MAXEVENT) {
    // Looks okay, address exists

    // Calculate sensor slot, if needed
    uint8_t sensor = (address >= 22 && address <= 29) ? 0 : 1;

    // Calculate DEVICECOND and comparison values, in case we need it.
    DEVICECOND dc = (DEVICECOND)(((value >> 14)) & 0x3);
    float fV = ((value & 0x0FFF) - 2048) / 10.0;

    // Examine address
    switch (address) {
    case 1: // Master switch
      settings.masterSwitch = (value ? true : false);
      registerEvent(settings.masterSwitch ? MASTER_ON : MASTER_OFF);
      break;
    case 20: // Measuring interval
      // In range?
      if (value >= 10 && value <= 3600) {
        settings.measuringInterval = value;
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 21: // Hysteresis steps
      // In range?
      if (value >= 1 && value <= 16) {
        settings.hystSteps = (value == 16 ? 0 : value);
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 22: // S0 type
    case 30: // S1 
      if (value < DEV_RESERVED) {
        settings.sensor[sensor].type = (DEVICEMODE)value;
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 23: // S0 Modbus IP bytes 0, 1
    case 31: // S1 
      settings.sensor[sensor].IP[0] = (value >> 8) & 0xFF;
      settings.sensor[sensor].IP[1] = value & 0xFF;
      break;
    case 24: // S0 Modbus IP bytes 2, 3
    case 32: // S1 
      settings.sensor[sensor].IP[2] = (value >> 8) & 0xFF;
      settings.sensor[sensor].IP[3] = value & 0xFF;
      break;
    case 25: // S0 Modbus port
    case 33: // S1 
      if (value) {
        settings.sensor[sensor].port = value;
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 26: // S0 Modbus SID and slot
    case 34: // S1 
      if (value) {
        uint8_t SID = (value >> 8) & 0xFF;
        uint8_t slot = value & 0xFF;
        if (SID && SID <= 247 && slot < 2) {
          settings.sensor[sensor].SID = SID;
          settings.sensor[sensor].slot = (slot ? true : false);
        } else {
          rc = ILLEGAL_DATA_VALUE;
        }
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 27: // S0 temperature condition and value
    case 35: // S1 
      if (dc != DEVC_RESERVED) {
        settings.sensor[sensor].TempMode = dc;
        settings.sensor[sensor].Temp = fV;
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 28: // S0 humidity condition and value
    case 36: // S1 
      if (dc != DEVC_RESERVED) {
        settings.sensor[sensor].HumMode = dc;
        settings.sensor[sensor].Hum = fV;
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 29: // S0 dew point condition and value
    case 37: // S1 
      if (dc != DEVC_RESERVED) {
        settings.sensor[sensor].DewMode = dc;
        settings.sensor[sensor].Dew = fV;
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 38: // Target type
      if (value < DEV_RESERVED) {
        settings.Target = (DEVICEMODE)value;
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 39: // Target Modbus IP bytes 0, 1
      settings.targetIP[0] = (value >> 8) & 0xFF;
      settings.targetIP[1] = value & 0xFF;
      break;
    case 40: // Target Modbus IP bytes 2, 3
      settings.targetIP[2] = (value >> 8) & 0xFF;
      settings.targetIP[3] = value & 0xFF;
      break;
    case 41: // Target Modbus port
      if (value) {
        settings.targetPort = value;
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 42: // Target Modbus SID
      if (value) {
        uint8_t SID = (value >> 8) & 0xFF;
        if (SID && SID <= 247) {
          settings.targetSID = SID;
        } else {
          rc = ILLEGAL_DATA_VALUE;
        }
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 43: // Combo temperature condition and value
      if (dc != DEVC_RESERVED) {
        settings.TempDiff = dc;
        settings.Temp = fV;
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 44: // Combo humidity condition and value
      if (dc != DEVC_RESERVED) {
        settings.HumDiff = dc;
        settings.Hum = fV;
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 45: // Combo dew point condition and value
      if (dc != DEVC_RESERVED) {
        settings.DewDiff = dc;
        settings.Dew = fV;
      } else {
        rc = ILLEGAL_DATA_VALUE;
      }
      break;
    case 47: // fallback switch
      settings.fallbackSwitch = (value ? true : false);
      break;
    default:
      // If we end up here, the address is not write-enabled
      rc = ILLEGAL_DATA_ADDRESS;
      break;
    }
  } else {
    // address outside register range
    rc = ILLEGAL_DATA_ADDRESS;
  }
  LOG_V("RC=%02X @%d: %04X\n", rc, address, value);
  return rc;
}

// Modbus server WRITE_HOLD_REGISTER callback
ModbusMessage FC06(ModbusMessage request) {
  ModbusMessage response;          // returned response message
  Error e = SUCCESS;               // Result value

  uint16_t address = 0;
  uint16_t value = 0;

  // Get address and new register value
  request.get(2, address);
  request.get(4, value);

  // Check address, data and write it in case all is OK
  e = writeRegister(address, value);

  // Generate appropriate response message
  if (e == SUCCESS) {
    response = ECHO_RESPONSE;
    // We need to write the settings!
    writeSettings();
    writeDeviceInfo();
  } else {
    response.setError(request.getServerID(), request.getFunctionCode(), e);
  }
  return response;
}

// Modbus server WRITE_MULT_REGISTERS callback
ModbusMessage FC10(ModbusMessage request) {
  ModbusMessage response;          // returned response message
  Error e = SUCCESS;               // Result value
  SetData backup = settings;       // Keep rollback data in case of errors
                                   // *** We are relying on default byte-wise copy here!

  uint16_t address = 0;
  uint16_t words = 0;
  uint16_t offs = 2;

  // Get address and new register value
  offs = request.get(offs, address);
  offs = request.get(offs, words);
  // Skip length byte
  offs++;

  // Valid address etc.?
  if (address && words && address + words <= 65 + MAXEVENT) {
    // Yes. Loop over words to be written
    for (uint16_t i = 0; i < words; i++) {
      // Get next value
      uint16_t value;
      offs = request.get(offs, value);
      // Check address, data and write it in case all is OK
      e = writeRegister(address + i, value);
      // Stop processing as soon as we have encountered an error
      if (e != SUCCESS) {
        break;
      }
    }
  } else {
    e = ILLEGAL_DATA_ADDRESS;
  }

  // Generate appropriate response message
  if (e == SUCCESS) {
    response.add(request.getServerID(), request.getFunctionCode(), address, words);
    // We need to write the settings!
    writeSettings();
    writeDeviceInfo();
  } else {
    response.setError(request.getServerID(), request.getFunctionCode(), e);
    // Roll back changes
    settings = backup;
  }
  return response;
}

// Reboot command
ModbusMessage FC44(ModbusMessage request) {
  ModbusMessage response;

  // Not yet armed?
  if (rebootPending == 0) {
    // No, init reboot sequence
    rebootPending = 1;
    rebootGrace = millis();
  } else {
    // Yes, was armed. Within grace period?
    if (millis() - rebootGrace < 60000) {
      // Yes, trigger reboot now
      rebootPending = 2;
    } else {
      // No, reset request
      rebootPending = 0;
    }
  }
  response.add(request.getServerID(), request.getFunctionCode(), (uint16_t)rebootPending);
  return response;
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
  WiFi.begin(settings.WiFiSSID, settings.WiFiPASS);

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

// Error handler for Modbus client
void handleError(Error e, uint32_t token) {
  ModbusError me(e);
  LOG_E("Error response for request %04X: %02X - %s\n", token, e, (const char *)me);
  registerMBerror(e);
  // Register it in the appropriate health tracker
  if (token == 0x1008) { 
    DHT0.healthTracker <<= 1; 
    DHT0.statusLED.start(DEVICE_ERROR_BLINK);
    DHT0.lastCheckOK = false;
  } else if ((token & 0xFFFF) == 0x1009) { 
    DHT1.healthTracker <<= 1; 
    DHT1.statusLED.start(DEVICE_ERROR_BLINK);
    DHT1.lastCheckOK = false;
  } else if ((token & 0xFFFF) == 0x2008 || (token & 0xFFFF) == 0x2009) { 
    targetHealth <<= 1; 
    targetLED.start(DEVICE_ERROR_BLINK);
  }
}

// Response handler for Modbus client
void handleData(ModbusMessage response, uint32_t token) {
  // Register successful request
  registerMBerror(SUCCESS);
  if ((token & 0xFFFF) == 0x1008 || (token & 0xFFFF) == 0x1009) { // Sensor data request
    // Get sensor slot
    mySensor& sensor = ((token & 0xFFFF) == 0x1008) ? DHT0 : DHT1;
    // get data
    uint16_t offs = 3;
    offs = response.get(offs, sensor.th.temperature);
    offs = response.get(offs, sensor.th.humidity);
    offs = response.get(offs, sensor.dewPoint);
    // Register successful request
    sensor.healthTracker <<= 1;
    sensor.healthTracker |= 1;
    sensor.statusLED.start(DEVICE_OK);
    sensor.lastCheckOK = true;
  } else if ((token & 0xFFFF) == 0x2008) { // target state request
    // Get data
    uint16_t stateT = 0;
    response.get(3, stateT);
    switchedON = (stateT > 0);
    // Register successful request
    targetHealth <<= 1;
    targetHealth |= 1;
    targetLED.start(DEVICE_OK);
  } else if ((token & 0xFFFF) == 0x2009) { // target switch request
    // Get data
    uint16_t stateT = 0;
    response.get(4, stateT);
    switchedON = (stateT > 0);
    // Register successful request
    targetHealth <<= 1;
    targetHealth |= 1;
    targetLED.start(DEVICE_OK);
    registerEvent(switchedON ? TARGET_ON : TARGET_OFF);
  } else {
    // Unknown token?
    LOG_E("Unknown response %04X received.\n", token);
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
      digitalWrite(TARGET_PIN, onOff ? HIGH : LOW);
      // Register event
      registerEvent(onOff ? TARGET_ON : TARGET_OFF);
      switchedON = onOff;
    } else if (settings.Target == DEV_MODBUS) {
      MBclient.setTarget(settings.targetIP, settings.targetPort);
      Error e = MBclient.addRequest((uint32_t)((millis() << 16) | 0x2009), settings.targetSID, WRITE_HOLD_REGISTER, 1, onOff ? 1 : 0);
      if (e != SUCCESS) {
        ModbusError me(e);
        LOG_E("Error sending 0x2009 request: %02X - %s\n", e, (const char *)me);
        registerMBerror(e);
      }
      LOG_V("Switch request sent\n");
    }
  }
  // Update signal LED anyway
  signalLED.start(onOff ? TARGET_ON_BLINK : TARGET_OFF_BLINK);
}

// Web server callbacks
// Illegal page requested
void notFound() {
  String message(256);
  message = "File Not Found\n\n";
  message += "URI: ";
  message += HTMLserver.uri();
  message += "\nMethod: ";
  message += (HTMLserver.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += HTMLserver.args();
  message += "\n";
  for (uint8_t i = 0; i < HTMLserver.args(); i++) { message += " " + HTMLserver.argName(i) + ": " + HTMLserver.arg(i) + "\n"; }
  HTMLserver.send(404, "text/plain", message);
  LOG_V("404 message=%d\n", message.length());
  HTMLserver.client().stop();
}

// Restart device
void handleRestart() {
  HTMLserver.client().stop();
  ESP.restart();
}

// Put out device status
void handleDevice() {
  String message(4096);
  
  // Use only in CONFIG mode
  if (mode == CONFIG) {
    message = "<!DOCTYPE html><html><header><link rel=\"stylesheet\" href=\"/styles.css\"><title>";
    if (*settings.deviceName) {
      message += settings.deviceName;
    } else {
      message += AP_SSID;
    }
    message += " status</title></header><body>\n";
    // Add in deviceinfo
    message += deviceInfo;
    message += "<button onclick=\"window.location.href='/config.html';\" class=\"button\"> CONFIG page </button><div class=\"divider\"/>";
    message += "<button onclick=\"window.location.href='/restart';\" class=\"button red-button\"> Restart </button></div>";
    message += "</body></html>";
    HTMLserver.send(200, "text/html", message);
    LOG_V("device message=%d\n", message.length());
    HTMLserver.client().stop();
  }
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
        if (needsWrite) {
          registerEvent(settings.masterSwitch ? MASTER_ON : MASTER_OFF);
        }
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
        if (uintval >= 1 && uintval != settings.targetPort) {
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
        if (uintval >= 1 && uintval != settings.sensor[sensor].port) {
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
      case 48: // Fallback switch
        if ((uintval == 0) && settings.fallbackSwitch) { settings.fallbackSwitch = false; needsWrite = true; }
        if ((uintval != 0) && !settings.fallbackSwitch) { settings.fallbackSwitch = true; needsWrite = true; }
        break;
      default: // Unhandled number
        LOG_I("CV parameter number unhandled [0..%d]: %d\n", CONFIGPARAMS, numbr);
        break;
      }
    } else {
      LOG_I("Unknown POST arg '%s'\n", cp);
    }
  }
  // If we found changes, report it.
  if (needsWrite) {
    writeSettings();
  }
  handleDevice();
}

void setup() {
  // Activate logging, if required
  MBUlogLvl = LOCAL_LOG_LEVEL;

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

  // Shut down target pin
  pinMode(TARGET_PIN, OUTPUT);
  digitalWrite(TARGET_PIN, LOW);

  // (Try to) init sensors
  DHT0.sensor.setup(SENSOR_0, DHTesp::DHT22);
  DHT1.sensor.setup(SENSOR_1, DHTesp::DHT22);

  // First check of sensors
  checkSensor(DHT0);
  checkSensor(DHT1);

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

  // Do we have a restarts file?
  if (LittleFS.exists(RESTARTS)) {
    // Yes. Open it for read
    File sF = LittleFS.open(RESTARTS, "r");
    // Successfully opened?
    if (sF) {
      // Yes. Read in settings struct
      sF.readBytes((char *)&restarts, sizeof(restarts));
      sF.close();
    } else {
      LOG_E("Settings file '" RESTARTS "' open failed.");
    }
  } else {
    LOG_E("Settings file '" RESTARTS "' does not exist.");
  }

  // Check if it is a valid settings file
  if (settings.magicValue == MAGICVALUE) {
    // It is - adjust runtime data with values from EEPROM
    // Increase boot count
    restarts++;
    // Write back
    File sF = LittleFS.open(RESTARTS, "w");
    if (sF) {
      sF.write((char *)&restarts, sizeof(restarts));
      sF.close();
    } else {
      LOG_E("Could not write '" RESTARTS "'");
    }
    // if we have neither WiFi access data nor a device name we need to go into CONFIG mode
    if (!*settings.deviceName || !*settings.WiFiPASS || !*settings.WiFiSSID) {
      mode = CONFIG;
    }
  } else {
    // No, fresh one, we need to initialize it
    settings.magicValue = MAGICVALUE;
    restarts = 0;
    settings.masterSwitch = false;
    settings.fallbackSwitch = false;
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

  LOG_I("Restarts=%d\n", restarts);

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

    // Start up OTA server
    ArduinoOTA.setHostname(settings.deviceName);  // Set OTA host name
    ArduinoOTA.setPassword((const char *)settings.OTAPass);  // Set OTA password
    ArduinoOTA.begin();               // start OTA scan

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
    MBclient.setTimeout(10000);
    // Register response handlers
    MBclient.onErrorHandler(handleError);
    MBclient.onDataHandler(handleData);

    // Register Modbus server functions
    MBserver.registerWorker(MYSID, READ_HOLD_REGISTER, FC03);
    MBserver.registerWorker(MYSID, WRITE_HOLD_REGISTER, FC06);
    MBserver.registerWorker(MYSID, WRITE_MULT_REGISTERS, FC10);
    // Special restart worker
    MBserver.registerWorker(MYSID, USER_DEFINED_44, FC44);

    // Create device info string
    writeDeviceInfo();

    // Start Modbus server
    MBserver.start(502, 4, 2000);

    signalLED.start(TARGET_OFF_BLINK);
  } else {
    // No, config mode
    // Start AP.
    WiFi.softAP(AP_SSID, "Maelstrom");

    // Set up open web server in CONFIG mode
    HTMLserver.on("/sub", handleSet);
    HTMLserver.on("/restart", handleRestart);
    // Set up mode independent web server callbacks
    HTMLserver.onNotFound(notFound);
    HTMLserver.on("/", handleDevice);
    HTMLserver.enableCORS(true);
    HTMLserver.serveStatic("/", LittleFS, "/");
    
    // Start web server
    HTMLserver.begin(80);

    // Signal config mode
    signalLED.start(CONFIG_BLINK);
  }
  LOG_I("%s, mode=%d\n", AP_SSID, mode);
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
  // Counter for measurement failures
  static uint16_t failCnt = 0;

  // Keep track of blinking status LEDs and button presses
  signalLED.update();
  targetLED.update();
  S0LED.update();
  S1LED.update();

  // Update mDNS
  MDNS.update();

  // Check OTA requests
  ArduinoOTA.handle();

  // RUN mode?
  if (mode == RUN) {
    if (tSwitch.update() > 0) {
      // Button pressed?
      ButtonEvent be = tSwitch.getEvent();
      // Short press?
      if (be == BE_CLICK) {
        // Yes, single click. Check sensors again
        checkSensor(DHT0);
        checkSensor(DHT1);
      } else if (be == BE_PRESS) {
        // No, button was held. Switch to manual mode
        signalLED.start(MANUAL_BLINK);
        S0LED.stop();
        S1LED.stop();
        targetLED.stop();
        mode = MANUAL;
        registerEvent(ENTER_MAN);
      } else if (be == BE_DOUBLECLICK) {
        // No again, we caught a double click instead
        // Let the three device LEDs signal the switch logic calculations result
        // Will be normalized as soon as the next measuring cycle is begun.
        // Sensor 0 all conditions met?
        if (s1cond == 3) { S0LED.start(MANUAL_BLINK);  // Yes, LED steady on
        } else           { S0LED.stop(); }             // No, LED off
        // Sensor 1 all conditions met?
        if (s2cond == 3) { S1LED.start(MANUAL_BLINK);
        } else           { S1LED.stop(); }
        // Combined conditions all met?
        if (cccond == 3) { targetLED.start(MANUAL_BLINK);
        } else           { targetLED.stop(); }
      }
    }
  
    // Is it time to advance the runtime counter?
    // (do a target state check as well)
    if (millis() - tick >= 60000) {
      // Yes, increment it as long as it did not hit the ceiling yet
      if (runTime < 65535) {
        runTime++;
      }
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
          MBclient.setTarget(settings.targetIP, settings.targetPort);
          Error e = MBclient.addRequest((uint32_t)((millis() << 16) | 0x2008), settings.targetSID, READ_HOLD_REGISTER, 1, 1);
          if (e != SUCCESS) {
            ModbusError me(e);
            Serial.printf("Error sending request 0x2008: %02X - %s\n", e, (const char *)me);
            registerMBerror(e);
          }
          LOG_V("Switch status requested\n");
        } else {
          // No, shall be local
          // We must trust on the wiring, so we assume good health
          switchedON = digitalRead(TARGET_PIN);
          targetHealth <<= 1;
          targetHealth |= 1;
          // Toggle LED to show target switch state
          targetLED.start(switchedON ? DEVICE_OK : DEVICE_IGNORED);
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
      // Kill oldest hysteresis bit
      Hysteresis <<= 1;
      // Check for valid measurements
      uint8_t measurementSuccess = 0;

      // Check both sensors
      for (uint8_t i = 0; i < 2; i++) {
        // Select sensor
        mySensor& sensor = (i == 0) ? DHT0 : DHT1;
        uint8_t& checks = (i == 0 ? s1cond : s2cond);
        // Keep in mind if the sensor is relevant at all
        sensor.isRelevant = (settings.sensor[i].TempMode != DEVC_NONE
              || settings.sensor[i].HumMode != DEVC_NONE
              || settings.sensor[i].DewMode != DEVC_NONE
              || settings.TempDiff != DEVC_NONE
              || settings.HumDiff != DEVC_NONE
              || settings.DewDiff != DEVC_NONE);

        // Do we need a measurement at all?
        if (settings.sensor[i].type != DEV_NONE) {
          // We do, check sensor.
          takeMeasurement(sensor);
          if (sensor.lastCheckOK) {
            measurementSuccess++;
          }
          // Is it relevant?
          if (sensor.isRelevant) {
            // Yes. Did we get data? (measurementSuccess will have been incremented already)
            if (sensor.lastCheckOK) {
              // Yes, we did.
              // 1: Check temperature
              switch (settings.sensor[i].TempMode) {
              case DEVC_NONE:  checks++; break;
              case DEVC_LESS:  if (sensor.th.temperature < settings.sensor[i].Temp) { checks++; } break;
              case DEVC_GREATER:  if (sensor.th.temperature > settings.sensor[i].Temp) { checks++; } break;
              case DEVC_RESERVED: break;
              }
              // 2: Check humidity
              switch (settings.sensor[i].HumMode) {
              case DEVC_NONE:  checks++; break;
              case DEVC_LESS:  if (sensor.th.humidity < settings.sensor[i].Hum) { checks++; } break;
              case DEVC_GREATER:  if (sensor.th.humidity > settings.sensor[i].Hum) { checks++; } break;
              case DEVC_RESERVED: break;
              }
              // 3: Check dew point
              switch (settings.sensor[i].DewMode) {
              case DEVC_NONE:  checks++; break;
              case DEVC_LESS:  if (sensor.dewPoint < settings.sensor[i].Dew) { checks++; } break;
              case DEVC_GREATER:  if (sensor.dewPoint > settings.sensor[i].Dew) { checks++; } break;
              case DEVC_RESERVED: break;
              }
            } else {
              // No, measurement has failed. Bail out here
              break;
            }
          } else {
            // It is irrelevant, so assume all conditions met
            checks = 3;
            // Additionally, no combo conditions will have to be met
            cccond = 3;
          }
        } else {
          // Sensor non-existent, assume all conditions met
          checks = 3;
          // Additionally, no combo conditions will have to be met
          cccond = 3;
          // Just in case clear previous LED signals
          sensor.statusLED.stop();
        }
      }

      // Finally check combo conditions
      // If we have one sensor only, no need to check
      if (cccond == 0) {
        // We have both sensors.
        // Did both measurements (if any) succeed?
        if (measurementSuccess == 2) {
          // Reset failure counter
          failCnt = 0;
          // Check temperature
          switch (settings.TempDiff) {
          case DEVC_NONE: cccond++; break;
          case DEVC_LESS: if (DHT0.th.temperature - DHT1.th.temperature < settings.Temp) { cccond++; } break;
          case DEVC_GREATER: if (DHT0.th.temperature - DHT1.th.temperature > settings.Temp) { cccond++; } break;
          case DEVC_RESERVED: break;
          }
          // Check humidity
          switch (settings.HumDiff) {
          case DEVC_NONE: cccond++; break;
          case DEVC_LESS: if (DHT0.th.humidity - DHT1.th.humidity < settings.Hum) { cccond++; } break;
          case DEVC_GREATER: if (DHT0.th.humidity - DHT1.th.humidity > settings.Hum) { cccond++; } break;
          case DEVC_RESERVED: break;
          }
          // Check dew point
          switch (settings.DewDiff) {
          case DEVC_NONE: cccond++; break;
          case DEVC_LESS: if (DHT0.dewPoint - DHT1.dewPoint < settings.Dew) { cccond++; } break;
          case DEVC_GREATER: if (DHT0.dewPoint - DHT1.dewPoint > settings.Dew) { cccond++; } break;
          case DEVC_RESERVED: break;
          }
        } else {
          // We failed for at least one sensor!
          failCnt++;
          if (failCnt > 3) {
            // Three failures in a row - fallback!
            switchTarget(settings.fallbackSwitch);
            registerEvent(FAIL_FB);
          }
        }
      }

      // Consider switching only if not in fail state
      if (failCnt == 0) {
        // All conditions met?
        if (s1cond + s2cond + cccond == 9) {
          Hysteresis |= 1;
        }
        // Store conditions for Modbus retrieval
        cState = (s1cond << 8) | (s2cond << 4) | cccond;

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

      // Collect data in history
      calcHistory.collect(DHT0.th.temperature, DHT0.th.humidity, DHT1.th.temperature, DHT1.th.humidity, switchedON);
        
      // Debug output
      LOG_V("S0 %5.1f %5.1f %5.1f %s\n", DHT0.th.temperature, DHT0.th.humidity, DHT0.dewPoint, DHT0.lastCheckOK ? "OK" : "FAIL");
      LOG_V("S1 %5.1f %5.1f %5.1f %s\n", DHT1.th.temperature, DHT1.th.humidity, DHT1.dewPoint, DHT1.lastCheckOK ? "OK" : "FAIL");
      LOG_V("    Check=%d/%d/%d Fails=%d Hysteresis=%04X\n", s1cond, s2cond, cccond, failCnt, Hysteresis);
      measure = millis();
    }
  
    // Reboot requested?
    if (rebootPending == 2) {
      // Yes. Restart now
      ESP.restart();
    } else if (rebootGrace && millis() - rebootGrace > 60000) {
      // No, but the grace period has passed. Deactivate reboot sequence
      rebootPending = 0;
      rebootGrace = 0;
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
        checkSensor(DHT0);
        checkSensor(DHT1);
        mode = RUN;
        registerEvent(EXIT_MAN);
      }
    }
  } else {
    // No, CONFIG mode!
    // Update web server
    HTMLserver.handleClient();
  }
}
