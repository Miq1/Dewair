// Copyright 2021 Michael Harwerth - miq1 AT gmx DOT de

#include <iostream>
#include <iomanip>
#include <regex>
#include <ctime>
#include "Logging.h"
#include "ModbusClientTCP.h"
#include "parseTarget.h"

using std::cout;
using std::endl;
using std::printf;
using std::hex;
using std::dec;

// DewAir data dump
struct DAdata {
  uint16_t masterSwitch;
  struct {
    float temp;
    float hum;
    float dew;
    uint16_t type;
    IPAddress ip;
    uint16_t port;
    uint8_t SID;
    uint8_t slot;
    uint16_t tCond;
    uint16_t hCond;
    uint16_t dCond;
    uint16_t health;
  } sensor[2];
  uint16_t targetType;
  IPAddress targetIP;
  uint16_t targetPort;
  uint8_t targetSID;
  uint16_t cTCond;
  uint16_t cHCond;
  uint16_t cDCond;
  uint16_t state;
  uint16_t restarts;
  uint16_t runTime;
  uint16_t tHealth;
  uint16_t interval;
  uint16_t steps;
  uint16_t eSlots;
  uint16_t hSlots;
  uint16_t hAddress;
  uint16_t hCurrent;
  uint16_t cState;
  uint16_t fallbackSwitch;
} d;

// History data
struct History {
  uint16_t t0;
  uint16_t h0;
  uint16_t t1;
  uint16_t h1;
  uint16_t on;
};

// Commands understood
const char *cmds[] = { 
  "INFO", "ON", "OFF", "EVERY", "EVENTS", "INTERVAL", "HYSTERESIS", 
  "TARGET", "SENSOR", "CONDITION", "FALLBACK", "REBOOT", "ERRORS",
  "HISTORY",
  "_X_END" };
enum CMDS : uint8_t { 
  INFO = 0, SW_ON, SW_OFF, EVRY, EVNTS, INTVL, HYST, 
  TRGT, SNSR, COND, FALLB, REBT, ERRS, HIST,
  X_END };

const char * typeNam[] = { "temperature", "humidity", "dew point", "reserved"};

void handleError(Error error, uint32_t token) 
{
  // ModbusError wraps the error code and provides a readable error message for it
  ModbusError me(error);
  cout << "Error response: " << (int)me << " - " << (const char *)me << " at " << token << endl;
}

void usage(const char *msg) {
  cout << msg << endl;
  cout << "Usage: DewAir host[:port[:serverID]]] [cmd [cmd_parms]]" << endl;
  cout << "  cmd: ";
  for (uint8_t c = 0; c < X_END; c++) {
    if (c) cout << " | ";
    cout << cmds[c];
  }
  cout << endl;
  cout << "  ON|OFF" << endl;
  cout << "  FALLBACK ON|OFF" << endl;
  cout << "  EVERY <seconds>" << endl;
  cout << "  EVENTS" << endl;
  cout << "  ERRORS" << endl;
  cout << "  HISTORY" << endl;
  cout << "  INTERVAL <seconds>" << endl;
  cout << "  HYSTERESIS <steps>" << endl;
  cout << "  TARGET NONE|LOCAL|<host[:port[:serverID]]]>" << endl;
  cout << "  SENSOR <0|1> NONE|LOCAL|<<host[:port[:serverID]]]> <0|1>>" << endl;
  cout << "  CONDITION <SENSOR <0|1>>|DIFF TEMP|HUM|DEW IGNORE|<BELOW|ABOVE <value>>" << endl;
  cout << "  REBOOT" << endl;
}

void printCond(const char *label, uint16_t cond, const char *label2) {
  uint8_t type = (cond >> 14) & 0x03;
  float val = int((cond & 0x0FFF) - 2048) / 10.0;
  char buf[20];

  if (type) {
    cout << " + " << label;
    if (type == 1) {
      cout << " below ";
    } else if (type == 2) {
      cout << " above ";
    } else {
      cout << " reserved? ";
    }
    snprintf(buf, 20, "%5.1f", val);
    cout << buf << label2 << endl;
  } else if (*label2 == 0) {
    cout << label << " is ignored" << endl;
  }
}

int writeSingleRegister(ModbusClient& MBclient, uint8_t targetServer, uint16_t addr, uint16_t iVal, uint16_t minVal, uint16_t maxVal, const char *label) {
  char buf[80];

//interval must be within limits
  if (iVal < minVal || iVal > maxVal) {
    snprintf(buf, 80, "%s requires a value [%d .. %d]", label, minVal, maxVal);
    usage(buf);
    return -1;
  }
  ModbusMessage response = MBclient.syncRequest(33, targetServer, WRITE_HOLD_REGISTER, addr, iVal);
  Error err = response.getError();
  if (err!=SUCCESS) {
    handleError(err, 33);
    return -1;
  } else {
    cout << "Done." << endl;
  }
  return 0;
}
  
// ============= main =============
int main(int argc, char **argv) {
  // Target host parameters
  IPAddress targetIP = NIL_ADDR;
  uint16_t targetPort = 502;
  uint8_t targetServer = 1;
  const uint16_t BUFLEN(128);
  char buf[BUFLEN];

  // Define a TCP client
  Client cl;
  cl.setNoDelay(true);

  // Check calling arguments.
  // Arg1 is mandatory, a host name or IP address, optionally followed by ":port number",
  // again optionally followed by ":server ID".
  if (argc < 2) {
    usage("At least one argument needed!\n");
    return -1;
  }

  if (int rc = parseTarget(argv[1], targetIP, targetPort, targetServer)) {
    usage("Target descriptor invalid!");
    return rc;
  }

  cout << "Using " << string(targetIP) << ":" << targetPort << ":" << (unsigned int)targetServer << endl;

  // Next shall be a command word. Omission is like INFO
  uint8_t cmd = X_END;
  if (argc > 2) {
    for (uint8_t c = INFO; c < X_END; c++) {
      if (strncasecmp(argv[2], cmds[c], strlen(cmds[c])) == 0) {
        cmd = c;
        break;
      }
    }
    if (cmd == X_END) {
      usage("Invalid command!");
      return -1;
    }
  } else {
    cmd = INFO;
  }
    
  // Define a Modbus client using the TCP client
  ModbusClientTCP MBclient(cl);

  // Set up ModbusTCP client.
  // Set message timeout to 2000ms and interval between requests to the same host to 200ms
  MBclient.setTimeout(2000, 200);
  // Start ModbusTCP background task
  MBclient.begin();

  // Set Modbus TCP server address and port number
  MBclient.setTarget(targetIP, targetPort);

  // Prepare master switch default (save one case structure)
  uint8_t masterVal = 0;

  switch (cmd) {
// --------- Get and print out state data ------------------
  case INFO:
    {
//    Read data as one block
      uint16_t addr = 1;
      uint16_t words = 64;
      uint16_t offs = 3;
      ModbusMessage response = MBclient.syncRequest(1, targetServer, READ_HOLD_REGISTER, addr, words);
      Error err = response.getError();
      if (err!=SUCCESS) {
        handleError(err, 1);
      } else {
        uint16_t temp;
        offs = response.get(offs, d.masterSwitch);
        offs = response.get(offs, d.sensor[0].temp);
        offs = response.get(offs, d.sensor[0].hum);
        offs = response.get(offs, d.sensor[0].dew);
        offs = response.get(offs, d.sensor[1].temp);
        offs = response.get(offs, d.sensor[1].hum);
        offs = response.get(offs, d.sensor[1].dew);
        offs = response.get(offs, d.state);
        offs = response.get(offs, d.restarts);
        offs = response.get(offs, d.runTime);
        offs = response.get(offs, d.sensor[0].health);
        offs = response.get(offs, d.sensor[1].health);
        offs = response.get(offs, d.tHealth);
        offs = response.get(offs, d.interval);
        offs = response.get(offs, d.steps);
        for (uint8_t i = 0; i < 2; i++) {
          offs = response.get(offs, d.sensor[i].type);
          for (uint8_t j = 0; j < 4; j++) {
            uint8_t b;
            offs = response.get(offs, b);
            d.sensor[i].ip[j] = b;
          }
          offs = response.get(offs, d.sensor[i].port);
          offs = response.get(offs, temp);
          d.sensor[i].SID = (temp >> 8) & 0xFF;
          d.sensor[i].slot = temp & 0xFF;
          offs = response.get(offs, d.sensor[i].tCond);
          offs = response.get(offs, d.sensor[i].hCond);
          offs = response.get(offs, d.sensor[i].dCond);
        }
        offs = response.get(offs, d.targetType);
        for (uint8_t j = 0; j < 4; j++) {
          uint8_t b;
          offs = response.get(offs, b);
          d.targetIP[j] = b;
        }
        offs = response.get(offs, d.targetPort);
        offs = response.get(offs, d.targetSID);
        offs++; // Skip zero byte
        offs = response.get(offs, d.cTCond);
        offs = response.get(offs, d.cHCond);
        offs = response.get(offs, d.cDCond);
        offs = response.get(offs, d.cState);
        offs = response.get(offs, d.fallbackSwitch);
        offs = response.get(offs, d.hSlots);
        offs = response.get(offs, d.hAddress);
        offs = response.get(offs, d.hCurrent);
        offs += 13 * 2; // Skip spare registers
        offs = response.get(offs, d.eSlots);

        // Print data
        snprintf(buf, BUFLEN, "Master switch %s", d.masterSwitch ? "ON" : "OFF");
        cout << buf << endl;
        snprintf(buf, BUFLEN, "Fallback: switch %s", d.fallbackSwitch ? "ON" : "OFF");
        cout << buf << endl;
        for (uint8_t i = 0; i < 2; i++) {
          if (d.sensor[i].type) {
            snprintf(buf, BUFLEN, "Sensor %d: %5.1f°C  %5.1f%%  %5.1f°C  - %04X", 
              i,
              d.sensor[i].temp,
              d.sensor[i].hum,
              d.sensor[i].dew,
              d.sensor[i].health);
            if (d.sensor[i].type == 2) {
              cout << buf;
              snprintf(buf, BUFLEN, " %d.%d.%d.%d:%d:%d slot %d", 
                d.sensor[i].ip[0],
                d.sensor[i].ip[1],
                d.sensor[i].ip[2],
                d.sensor[i].ip[3],
                d.sensor[i].port,
                d.sensor[i].SID,
                d.sensor[i].slot);
            }
            cout << buf << endl;
          }
        }
        switch (d.targetType) {
        case 0: // none
          cout << "No target defined." << endl;
          break;
        case 1:
        case 2:
          snprintf(buf, BUFLEN, "Target is %s - %04X", d.state ? "ON" : "OFF", d.tHealth);
          if (d.targetType == 2) {
            cout << buf;
            snprintf(buf, BUFLEN, " %d.%d.%d.%d:%d:%d", 
              d.targetIP[0],
              d.targetIP[1],
              d.targetIP[2],
              d.targetIP[3],
              d.targetPort,
              d.targetSID);
          } 
          cout << buf << endl;
          break;
        }
        snprintf(buf, BUFLEN, "Measuring every %d seconds, hysteresis steps: %d", d.interval, d.steps);
        cout << buf << endl;
        snprintf(buf, BUFLEN, "%d restarts, run time since last restart %d:%02d", 
          d.restarts,
          d.runTime / 60,
          d.runTime % 60);
        cout << buf << endl;
        snprintf(buf, BUFLEN, "%d event slots", d.eSlots);
        cout << buf << endl;
        snprintf(buf, BUFLEN, "%d history slots starting at %u, current is %u", d.hSlots, d.hAddress, d.hCurrent);
        cout << buf << endl;
        cout << "Switch conditions (all must be true): " << endl;
        uint8_t sensCnt = 0;
        for (uint8_t i = 0; i < 2; i++) {
          if (d.sensor[i].type) {
            sensCnt++;
            snprintf(buf, BUFLEN, "S%d temperature", i);
            printCond(buf, d.sensor[i].tCond, "°C");
            snprintf(buf, BUFLEN, "S%d humidity", i);
            printCond(buf, d.sensor[i].hCond, "%");
            snprintf(buf, BUFLEN, "S%d dew point", i);
            printCond(buf, d.sensor[i].dCond, "°C");
          }
        }
        if (sensCnt >= 2) {
          printCond("temperature difference S0 - S1", d.cTCond, "°C");
          printCond("humidity difference S0 - S1", d.cHCond, "%");
          printCond("dew point difference S0 - S1", d.cDCond, "°C");
        }
        snprintf(buf, BUFLEN, "Condition state: %04X", d.cState);
        cout << buf << endl;
      }
    }
    break;
// --------- Print measurements in a loop ------------------
  case EVRY:
    {
//    Get seconds interval
      int interval;
      if (argc > 3) {
        interval = atoi(argv[3]);
        if (interval < 60) {
          usage("EVERY interval must not be below 60 seconds.");
          return -1;
        }
        cout << "Time;S0;;;S1;;;Diff;;;target" << endl;
        cout << ";temp;hum;dew;temp;hum;dew;temp;hum;dew;" << endl;
        while(1) { // loop forever...
//        Read data as one block
          time_t now = time(NULL);
          struct tm *tm = localtime(&now);

          uint16_t addr = 1;
          uint16_t words = 14;
          uint16_t offs = 3;
          ModbusMessage response = MBclient.syncRequest(21, targetServer, READ_HOLD_REGISTER, addr, words);
          Error err = response.getError();
          if (err!=SUCCESS) {
            handleError(err, 1);
          } else {
            offs = response.get(offs, d.masterSwitch);
            for (uint8_t i = 0; i < 2; i++) {
              offs = response.get(offs, d.sensor[i].temp);
              offs = response.get(offs, d.sensor[i].hum);
              offs = response.get(offs, d.sensor[i].dew);
            }
            offs = response.get(offs, d.state);
            snprintf(buf, BUFLEN, "%2d:%02d;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%.1f;%d",
              tm->tm_hour,
              tm->tm_min,
              d.sensor[0].temp,
              d.sensor[0].hum,
              d.sensor[0].dew,
              d.sensor[1].temp,
              d.sensor[1].hum,
              d.sensor[1].dew,
              d.sensor[0].temp - d.sensor[1].temp,
              d.sensor[0].hum - d.sensor[1].hum,
              d.sensor[0].dew - d.sensor[1].dew,
              d.state ? 1 : 0);
            cout << buf << endl;
          }
          sleep(interval);
        }
      } else {
//    Seconds parameter missing
        usage("EVERY needs a number of seconds (interval)");
        return -1;
      }
    }
    break;
// --------- Switch ON ------------------
  case SW_ON:
    masterVal = 1;
    // Fall-through intended!
    [[fallthrough]];
// --------- Switch OFF ------------------
  case SW_OFF:
    {
//    address 1, value between 0 and 1, name ON/OFF
      return writeSingleRegister(MBclient, targetServer, 1, masterVal, 0, 1, "ON/OFF");
    }
    break;
// --------- Read event storage -----------------
  case EVNTS:
    {
//    Read number of event slots
      uint16_t addr = 64;
      uint16_t words = 1;
      uint16_t offs = 3;
      ModbusMessage response = MBclient.syncRequest(18, targetServer, READ_HOLD_REGISTER, addr, words);
      Error err = response.getError();
      if (err!=SUCCESS) {
        handleError(err, 18);
      } else {
        uint16_t events = 0;
        offs = response.get(offs, events);
//      Has it some?
        if (events) {
//        Yes. Read them.
          addr = 65;
          offs = 3;
          response = MBclient.syncRequest(19, targetServer, READ_HOLD_REGISTER, addr, events);
          err = response.getError();
          if (err!=SUCCESS) {
            handleError(err, 19);
          } else {
            cout << events << " event slots found." << endl;
//          We got some. Print those that have a meaning
            uint16_t word = 0;
            uint8_t ev = 0;
            uint8_t hi = 0;
            uint8_t lo = 0;
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
            //          Loop over result data
            for (uint16_t i = 0; i < events; i++) {
              offs = response.get(offs, word);
              ev = (word >> 11) & 0x1F;
              hi = (word >> 6) & 0x1F;
              lo = word & 0x3F;
              if (ev != NO_EVENT) {
                if (ev == DATE_CHANGE || ev == BOOT_DATE) {
                  snprintf(buf, BUFLEN, "%-15s %02d.%02d.", eventname[ev], hi, lo);
                } else {
                  snprintf(buf, BUFLEN, "%-20s %02d:%02d", eventname[ev], hi, lo);
                }
                cout << buf << endl;
              }
            }
          }
        } else {
          cout << "Device has no events." << endl;
          return 0;
        }
      }
    }
    break;
// --------- measuring interval ------------------
  case INTVL:
    {
//    Try to get the parameter
      uint16_t iVal = 0;
      if (argc > 3) {
        iVal = atoi(argv[3]);
      }
//    address 20, value between 10 and 3600, name INTERVAL
      return writeSingleRegister(MBclient, targetServer, 20, iVal, 10, 3600, "INTERVAL");
    }
    break;
// --------- hysteresis steps ------------------
  case HYST:
    {
//    Try to get the parameter
      uint16_t iVal = 0;
      if (argc > 3) {
        iVal = atoi(argv[3]);
      }
//    address 21, value between 1 and 16, name HYSTERESIS
      return writeSingleRegister(MBclient, targetServer, 21, iVal, 1, 16, "HYSTERESIS");
    }
    break;
// --------- target definition -----------------
  case TRGT:
    {
//    We have three options: NONE, LOCAL or a Modbus address
      if (argc <= 3) {
        usage("TARGET needs one parameter");
        return -1;
      }
//    Is it NONE?
      if (strncasecmp(argv[3], "NONE", 4) == 0) {
//      address 38, value 0, name TARGET
        return writeSingleRegister(MBclient, targetServer, 38, 0, 0, 2, "TARGET:NONE");
//    No, but LOCAL?
      } else if (strncasecmp(argv[3], "LOCAL", 5) == 0) {
//      address 38, value 1, name TARGET
        return writeSingleRegister(MBclient, targetServer, 38, 1, 0, 2, "TARGET:LOCAL");
//    No, so it must be a Modbus target
      } else {
        IPAddress myIP;
        uint16_t myPort;
        uint8_t mySID;
        if (int rc = parseTarget(argv[3], myIP, myPort, mySID)) {
          usage("TARGET Modbus address invalid!");
          return rc;
        }
//      We need to shift the SID left for compatibility to sensor addresses
        uint16_t myShiftedSID = (mySID << 8);
//      Set up request message
        ModbusMessage request;
//      Header: SID, function code, address, words, bytes
        request.add(targetServer, WRITE_MULT_REGISTERS, (uint16_t)38, (uint16_t)5, (uint8_t)10);
//      Add target data
        request.add((uint16_t)2);  // type
        for (uint8_t i = 0; i < 4; i++) {
          request.add(myIP[i]);    // IP byte
        }
        request.add(myPort, myShiftedSID); // SID
//      Send request
        ModbusMessage response = MBclient.syncRequest(request, (uint32_t)20);
        Error err = response.getError();
        if (err!=SUCCESS) {
          handleError(err, 20);
          return -1;
        } else {
          cout << "Done." << endl;
        }
      }
    }
    break;
// --------- SENSOR definition -----------------
  case SNSR:
    {
//    We need a sensor number and one of three options: NONE, LOCAL or a Modbus address plus slot
      if (argc <= 4) {
        usage("SENSOR needs two parameters minimum");
        return -1;
      }
// Get the sensor index
      uint8_t sensNum = atoi(argv[3]);
      if (sensNum > 1) {
        usage("SENSOR number must be 0 or 1");
        return -1;
      }
//    Get type
//    Is it NONE?
      if (strncasecmp(argv[4], "NONE", 4) == 0) {
//      address 22, value 0, name SENSOR 0
//      address 30, value 0, name SENSOR 1
        snprintf(buf, BUFLEN, "SENSOR %d:NONE", sensNum);
        return writeSingleRegister(MBclient, targetServer, sensNum ? 30 : 22, 0, 0, 2, buf);
//    No, but LOCAL?
      } else if (strncasecmp(argv[4], "LOCAL", 5) == 0) {
//      address 22, value 1, name SENSOR 0
//      address 30, value 1, name SENSOR 1
        snprintf(buf, BUFLEN, "SENSOR %d:LOCAL", sensNum);
        return writeSingleRegister(MBclient, targetServer, sensNum ? 30 : 22, 1, 0, 2, buf);
//    No, so it must be a Modbus target
      } else {
//      We need a slot parameter as well, so check that first
        uint8_t slotNum = 99;
        if (argc >= 5) {
          slotNum = atoi(argv[5]);
        }
        if (slotNum > 1) {
          usage("SENSOR <n> <Modbus> needs a slot number <0|1> as well");
          return -1;
        }
        IPAddress myIP;
        uint16_t myPort;
        uint8_t mySID;
        if (int rc = parseTarget(argv[4], myIP, myPort, mySID)) {
          usage("SENSOR Modbus address invalid!");
          return rc;
        }
//      We need to shift the SID left and OR-in the slot number
        uint16_t myShiftedSID = (mySID << 8) | slotNum;
//      Set up request message
        ModbusMessage request;
//      Header: SID, function code, address, words, bytes
        request.add(targetServer, WRITE_MULT_REGISTERS, (uint16_t)(sensNum ? 30 : 22), (uint16_t)5, (uint8_t)10);
//      Add target data
        request.add((uint16_t)2);  // type
        for (uint8_t i = 0; i < 4; i++) {
          request.add(myIP[i]);    // IP byte
        }
        request.add(myPort, myShiftedSID); // SID
//      Send request
        ModbusMessage response = MBclient.syncRequest(request, (uint32_t)22);
        Error err = response.getError();
        if (err!=SUCCESS) {
          handleError(err, 22);
          return -1;
        } else {
          cout << "Done." << endl;
        }
      }
    }
    break;
// --------- Switching conditions -----------------
  case COND:
    {
//    We will need 3 arguments at least
      if (argc <= 5) {
        usage("CONDITION needs 3 parameters minimum");
        return -1;
      }
      uint8_t nextArg = 3;
      uint16_t offset = 43; // Default: first combo condition register
      uint8_t sensNum = 99;
//    Get reference - sensor or combination
      if (strncasecmp(argv[nextArg], "SENSOR", 6) == 0) {
//      Seems to be SENSOR. get the number next
        nextArg++;
        sensNum = atoi(argv[nextArg]);
        if (sensNum > 1) {
          usage("SENSOR number must be 0 or 1");
          return -1;
        }
        offset = 27 + sensNum * 8;  // first condition register for the chosen sensor
      } else if (strncasecmp(argv[nextArg], "DIFF", 4) == 0) {
//      Not SENSOR, but DIFF
//      Nothing to be done here, as it is the default
      } else {
//      Nothing we understand here
        usage("CONDITION 1st parameter must be SENSOR or DIFF");
        return -1;
      }
      nextArg++;
//    Next must be one of TEMP, HUM or DEW
      uint8_t type = 99;
      if (strncasecmp(argv[nextArg], "TEMPERATURE", strlen(argv[nextArg])) == 0) {
        type = 0;
      } else if (strncasecmp(argv[nextArg], "HUMIDITY", strlen(argv[nextArg])) == 0) {
        type = 1;
      } else if (strncasecmp(argv[nextArg], "DEWPOINT", strlen(argv[nextArg])) == 0) {
        type = 2;
      }
//    Everything else is invalid
      if (type > 2) {
        usage("CONDITION must use TEMPERATURE, HUMIDITY or DEWPOINT criteria");
        return -1;
      }
      nextArg++;
//    We have checked for 4 parameters only, so we may have run out of further
      if (nextArg >= argc) {
        usage("At least one more parameter required");
        return -1;
      }
//    Get condition type
      uint8_t cType = 99;
      if (strncasecmp(argv[nextArg], "IGNORE", strlen(argv[nextArg])) == 0) {
        cType = 0;
      } else if (strncasecmp(argv[nextArg], "BELOW", strlen(argv[nextArg])) == 0) {
        cType = 1;
      } else if (strncasecmp(argv[nextArg], "ABOVE", strlen(argv[nextArg])) == 0) {
        cType = 2;
      }
//    Everything else is invalid
      if (cType > 2) {
        usage("CONDITION must use IGNORE, BLOW or ABOVE");
        return -1;
      }
//    BELOW/ABOVE need a float argument as well
      uint16_t uVal = 0;
      if (cType > 0) {
        nextArg++;
        if(nextArg >= argc) {
          usage("BELOW and ABOVE need a value");
          return -1;
        }
//      Check if it is within bounds
        float cVal = atof(argv[nextArg]);
        uVal = (int(cVal * 10) + 2048) & 0x0FFF;
        if (abs(float(uVal-2048)/10.0 - cVal) > 0.05) {
          usage("CONDITION values may only be between -204.7 and 204.7");
          return -1;
        }
      }
//    combine type and value
      uVal |= (cType << 14);
//    Adjust offset
      offset += type;
//    Send request
      ModbusMessage response = MBclient.syncRequest(23, targetServer, WRITE_HOLD_REGISTER, offset, uVal);
      Error err = response.getError();
      if (err!=SUCCESS) {
        handleError(err, 23);
        return -1;
      } else {
        if (sensNum < 2) {
          snprintf(buf, BUFLEN, "Sensor %d %s condition:", sensNum, typeNam[type]);
        } else {
          snprintf(buf, BUFLEN, "Difference %s condition:", typeNam[type]);
        }
        printCond(buf, uVal, "");
        cout << "Done." << endl;
      }
    }
    break;
// --------- fallback policy ------------------
  case FALLB:
    {
//    We need ON or OFF as a parameter
      if (argc <= 3) {
        usage("FALLBACK needs ON or OFF");
        return -1;
      }
//    Try to get the parameter
      uint16_t iVal = 99;
      if (strncasecmp(argv[3], "ON", 2) == 0) {
        iVal = 1;
      } else if (strncasecmp(argv[3], "OFF", 3) == 0) {
        iVal = 0;
      }
      if (iVal == 99) {
        usage("FALLBACK needs ON or OFF");
        return -1;
      }
//    address 47, value 1 or 0, name FALLBACK
      return writeSingleRegister(MBclient, targetServer, 47, iVal, 0, 1, "FALLBACK");
    }
    break;
// --------- trigger reboot ------------------
  case REBT:
    {
//    Function code 44
//    Send request
      ModbusMessage response = MBclient.syncRequest(99, targetServer, USER_DEFINED_44);
      Error err = response.getError();
      if (err!=SUCCESS) {
        handleError(err, 99);
        return -1;
      } else {
        uint16_t uval = 99;
        response.get(2, uval);
        switch (uval) {
        case 0: // missed grace interval
          cout << "disarmed." << endl;
          break;
        case 1: // Initiated
          cout << "armed." << endl;
          break;
        case 2: // rebooting
          cout << "rebooting." << endl;
          break;
        default:
          cout << "unknown state." << endl;
          break;
        }
      }
    }
    break;
// --------- Read error tracking storage -----------------
  case ERRS:
    {
//    Read number of event slots to get offset to error tracking data
      uint16_t addr = 64;
      uint16_t words = 1;
      uint16_t offs = 3;
      ModbusMessage response = MBclient.syncRequest(24, targetServer, READ_HOLD_REGISTER, addr, words);
      Error err = response.getError();
      if (err!=SUCCESS) {
        handleError(err, 24);
      } else {
        // Got that. Now get number of error slots
        uint16_t errOffs = 0;
        response.get(offs, errOffs);
        addr += errOffs + 1;
        response = MBclient.syncRequest(25, targetServer, READ_HOLD_REGISTER, addr, words);
        err = response.getError();
        if (err!=SUCCESS) {
          handleError(err, 25);
        } else {
          uint16_t errCnt = 0;
          response.get(offs, errCnt);
          // Are there any?
          if (errCnt) {
            // Yes. go get them
            addr++;
            words = errCnt * 2;
            response = MBclient.syncRequest(26, targetServer, READ_HOLD_REGISTER, addr, words);
            err = response.getError();
            if (err!=SUCCESS) {
              handleError(err, 26);
            } else {
              // We seem to have some. Print out
              cout << "Back : ERR  Count" << endl;
              for (uint16_t inx = 0; inx < errCnt; ++inx) {
                uint16_t code, count;
                offs = response.get(offs, code);
                offs = response.get(offs, count);
                if (count) {
                  ModbusError me((Error)code);
                  snprintf(buf, BUFLEN, "%5u:  %02X  %5u - %s\n", inx, code, count, (const char *)me);
                  cout << buf;
                }
              }
            }
          } else {
            cout << "no errors tracking data available." << endl;
          }
        }
      }
    }
    break;
// --------- history data ------------------
  case HIST:
    {
//    Get relevant parameters first
      uint16_t addr = 48;
      uint16_t words = 3;
      uint16_t offs = 3;
      ModbusMessage response = MBclient.syncRequest(27, targetServer, READ_HOLD_REGISTER, addr, words);
      Error err = response.getError();
      if (err!=SUCCESS) {
        handleError(err, 27);
      } else {
        // Got parameters. Check and in case allocate memory
        uint16_t hSlots, hAddress, hCurrent;
        offs = response.get(offs, hSlots, hAddress, hCurrent);
        cout << "slots=" << hSlots << ", address=" << hAddress << ", current=" << hCurrent << endl;
        History h[hSlots];
        // Read data in blocks
        // ***** for now only read up to 126 slots! *****
        addr = hAddress;
        words = hSlots;
        for (uint8_t block = 0; block < 5; block++) {
          response = MBclient.syncRequest(28 + block, targetServer, READ_HOLD_REGISTER, addr, words);
          err = response.getError();
          if (err!=SUCCESS) {
            handleError(err, 28 + block);
          } else {
            // Got data. Sort it into the right box
            offs = 3;
            for (uint16_t i = 0; i < hSlots; i++) {
              switch (block) {
              case 0: // sensor 0 temp
                offs = response.get(offs, h[i].t0);
                break;
              case 1: // sensor 0 hum
                offs = response.get(offs, h[i].h0);
                break;
              case 2: // sensor 1 temp
                offs = response.get(offs, h[i].t1);
                break;
              case 3: // sensor 1 hum
                offs = response.get(offs, h[i].h1);
                break;
              case 4: // ON percentage
                offs = response.get(offs, h[i].on);
                break;
              default: // cannot happen...
                break;
              }
            }
          }
          addr += hSlots;
        }
        // Got everything now
        uint16_t minPerSlot = 1440 / hSlots;
        cout << "Time;S0 temp;S0 hum;S1 temp;S1 hum;Target ON;now" << endl;
        for (uint16_t i = 0; i < hSlots; i++) {
          snprintf(buf, BUFLEN, 
            "%2u:%02u;%.1f;%.1f;%.1f;%.1f;%u;%c",
            (i * minPerSlot) / 60,
            (i * minPerSlot) % 60,
            h[i].t0 ? h[i].t0 / 10.0 - 100.0 : 0.0,
            h[i].h0 ? h[i].h0 / 10.0 : 0.0,
            h[i].t1 ? h[i].t1 / 10.0 - 100.0 : 0.0,
            h[i].h1 ? h[i].h1 / 10.0 : 0.0,
            h[i].on,
            i == hCurrent ? '#' : ' '
          );
          cout << buf << endl;
        }
      }
    }
    break;
  default:
    usage("MAYNOTHAPPEN error?!?");
    return -2;
  }
// The happy end! ;)
  return 0;
}

