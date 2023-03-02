# Extras
This folder contains additional components and materials I developed for my DewAir devices - namely, the dew point ventilation controller and the frost protection device.

- Enclosure contains FreeCAD 3D designs and STL files for a small and a large box. The small one is powered by USB or 5V and needs a WiFi socket as target device, the larger box has all inside to run autonomously when connected to a wall power outlet.
- Fan Mount has 3D FreeCAD designs and STLs as well for attaching a 100mm standard tube ventilator or a 100mm air gauge to a window pane. I replaced the glass panes by acrylic glass and cut a hole in in to fit the mount. A drilling/cutting template can be 3D printed as well.
- PCB holds all KiCAD and Gerber files necessary to have the PCB manufactured.
- ``DewAir.cpp`` is a Linux command line tool to configure and control any DewAir device. It requires the Linux port of the [eModbus](https://github.com/eModbus/eModbus) library to be built - found there in the [examples/Linux](https://github.com/eModbus/eModbus/tree/master/examples/Linux) directory.
See below for build instructions etc.

### DewAir Linux tool
All interaction with the device is done by Modbus TCP. 
You of course may use other Modbus TCP clients to achieve the same, but you will need to closely inspect the Modbus register list for it.
The ``DewAir`` tool has all the definitions built in and lets you use plain text words to interact with the device.

#### Build and Install
As a prerequisite you will need to build the Linux eModbus library as mentioned in the first section.
With that available you can compile the program by
```
g++ DewAir.cpp -Wextra -DLOG_LEVEL=3 -MMD -leModbus -pthread -lexplain -o DewAir
```
#### Usage
Running ``DewAir`` without any parameter will display the supported options:
```
micha@LinuxBox:~$ DewAir 
At least one argument needed!

Usage: DewAir host[:port[:serverID]]] [cmd [cmd_parms]]
  cmd: INFO | ON | OFF | EVERY | EVENTS | INTERVAL | HYSTERESIS | TARGET | SENSOR | CONDITION | FALLBACK | REBOOT | ERRORS | HISTORY
  ON|OFF
  FALLBACK ON|OFF
  EVERY <seconds>
  EVENTS
  ERRORS
  HISTORY
  INTERVAL <seconds>
  HYSTERESIS <steps>
  TARGET NONE|LOCAL|<host[:port[:serverID]]]>
  SENSOR <0|1> NONE|LOCAL|<<host[:port[:serverID]]]> <0|1>>
  CONDITION <SENSOR <0|1>>|DIFF TEMP|HUM|DEW IGNORE|<BELOW|ABOVE <value>>
  REBOOT
```
``DewAir`` needs a device as first parameter in any case.
This can be the DNS name the device has been assigned, or a detailed address consisting of an IP address, a port number and a Modbus server ID, separated by colons (':').
If omitted, the server ID defaults to 1, the port number is 502 by default.

After the address of the device a command is expected. If none was given, ``INFO`` is taken as a default.

#### INFO
With ``INFO`` the current data is requested from the device. Sample output:
```
micha@LinuxBox:~$ DewAir anbau 
Using 192.168.178.30:502:1
Master switch ON
Fallback: switch ON
Sensor 0:  12.1°C   56.0%    3.6°C  - FFFF
Sensor 1:   6.4°C   45.8%   -4.4°C  - FFFF
Target is OFF - FFFF 192.168.178.40:502:1
Measuring every 30 seconds, hysteresis steps: 4
86 restarts, run time since last restart 24:48
40 event slots
120 history slots starting at 400, current is 83
Switch conditions (all must be true): 
 + S0 temperature below   5.0°C
Condition state: 0233
```
You see how the DNS name ``anbau`` is translated internally to a full address.
The device is active (master is ON), on failures it will switch the target to ON.
Both sensors are connected locally and are performing well (``FFFF`` tells us the last 16 sensor requests were successful).
The target device is a Modbus-controlled remote socket, it is currently OFF and communication was okay (``FFFF`` again).
Sensors are requested every 30 seconds and 4 consecutive identical evaluations of the conditions set will trigger a switching action.
The device was restarted 86 times and is running more than a day now after the last restart.
It does register the last 40 events (see below) and the data history has 120 time slices over a day. Modbus register 400 holds the first history value.
The device is set to switch the target as soon as sensor 0 will report a temperature below 5 degrees. Current temperature is above that, so the state is not ``0333`` as it would be with all conditions met.

#### ON/OFF
This will activate or deactivate the device. The 'Master switch' is turned ON or OFF.
If set to OFF, the device will work as normal, but never will touch the target's state.

#### FALLBACK
If sensors have malfunctions or are not responding, there is no reliable evaluation of the switch conditions possible any more.
``FALLBACK`` defines the target state that will be set in such situations.
F.i. a frost protection device may set to fall back to ON state to prevent freezing - for the cost of unnecessary energy wasted.

#### EVERY
``EVERY`` will request and print out the sensor and target state values in the intervals given as additional seconds parameter.
Note that this value must be 60 or higher to prevent the device being flooded with requests.
The output is a comma separated list for easy processing in a spreadsheet program. To have an output readable on the terminal you should pipe it through a ``tr`` like in
```
micha@LinuxBox:~$ DewAir anbau every 60 | tr ';' '\t'
Using 192.168.178.30:502:1
Time    S0                      S1                      Diff                    target
        temp    hum     dew     temp    hum     dew     temp    hum     dew
17:25   12.2    53.0    2.9     5.6     47.2    -4.8    6.6     5.8     7.7     0
```
Output can be stopped by ``Ctrl C`` only.

#### EVENTS
Important things happening will be recorded on the device in the ``EVENTS`` list. It looks like
```
micha@LinuxBox:~$ DewAir anbau events
Using 192.168.178.30:502:1
40 event slots found.
target off           18:06
target off           20:05
target off           20:06
target off           22:05
target off           22:06
date change     01.03.
target off           00:05
target off           00:06
target off           01:09
target off           02:05
target off           04:05
target on            04:49
target off           04:54
target on            05:28
...
```
(Note: in this example the target itself is programmed to turn on every two hours, so as soon as the DewAir device notices the target being ON, it will switch it OFF again).

#### ERRORS
Similar to the events, Modbus errors are recorded on the device.
With the ``ERRORS`` command you may retrieve what was stored:
```
micha@LinuxBox:~$ DewAir anbau errors
Using 192.168.178.30:502:1
Back : ERR  Count
    0:  00    229 - Success
    1:  E0      1 - Timeout
    2:  00    410 - Success
    3:  EA      1 - IP connection failed
    4:  00    937 - Success
```
Only if the Modbus state has changed a new entry is made. Else only the number of occurrences of the same state will be counted.
Note that the most recent state is shown on top, the lower, the older the stes are.

#### HISTORY
All sensor data and target states are collected for 24h. Values older will be replaced by current values as time proceeds.
The data is averaged over time slots. A slot is 12 minutes wide in the default code.
Hence you will have 120 slots per 24h that can be retrieved in a single Modbus request.
There are 120 such values for sensor 0 temperature, humidity, sensor 1 temperature and humidity and the target state value. 
A complete history data set will have 600 values, each in a Modbus register.
Floating point sensor values are encoded to fit into the 16 bits of a Modbus register.
The coding scheme is described on the main page - see section 'History' there.

The ``HISTORY`` command will do all 5 requests and the decoding in one step and outputs the data as a comma-separated list to be processed in a spreadsheet program.
It is advisable to catch the output in a file and open it in a spreadsheet:
```
micha@LinuxBox:~$ DewAir anbau history > anbau.csv
```
With a bit of massaging in the spreadsheet you can quickly get some nice diagrams:

<img src=https://github.com/Miq1/DewAir/blob/master/Extras/Diagram.png alt="Sample diagram">

A sixth column is added to the output that is almost empty. The exception is a ``#`` sign in that row where the next slot will be written.
So right of that line is data from the previous day, left of it the data for the current day up to now.

In the diagram above I used a simple formula (``=IF($H3=" ";$G2;100)``) to have a line jump up at the current slot. It will take a zero as start value and always replicate the value of the line above - except if the neighbouring cell has something other than a blank in it; then the value will set to 100.

#### REBOOT
A mechanism has been integrated to remotely restart a DewAir device. 
It makes use of the user-definable Modbus function code 0x44.
``REBOOT`` has to be sent twice within 60 seconds to have an effect. 
This is meant as a basic security mechanism. 
The first ``REBOOT`` will be answered by ``armed.``, the second, if received within the 60s period, will print out ``rebooting.``.
If the second commend is late or not given at all, the ``armed`` state will be reset on the device.

#### INTERVAL ``<seconds>`` and HYSTERESIS ``<turns>``
These two commands will modify the reaction speed of the device on changing sensor data.
Data is sampled every ``INTERVAL`` seconds. Note that the seconds parameter can not be lower than 20 to not overload the device.

``HYSTERESIS`` sets a number of sampling periods where the sampled values are fulfilling the switching conditions without exception.
If that happens, a switch is executed.

F.i. a combination of 30 seconds and a hysteresis of 6 will require the switch conditions to last at least 3 minutes to trigger a switch.
Higher values will lead to a slower, smoother switching behaviour, whereas short intervals or less hysteresis periods will let the device react more quickly.
It depends from your application which may be the adequate setting.

#### TARGET
The ``TARGET`` command lets you define if there is a switching target at all and what type it is.

A ``TARGET NONE`` will disable any switching actions, while sensor data still is collected.
This may be useful if you are using a DewAir device as a remote temperature/humidity platform to read these data and make it available to other devices.
Please see the ``SENSOR`` explanations in the section below.

``TARGET LOCAL`` enables the device to use the internal relay/SSR logic as a target. 
A stand-alone DewAir device may switch a built-in socket this way.

Finally ``TARGET <Modbus address>`` allows for remote target devices that can be switched by a Modbus request.
At this time only the [Smartdose](https://github.com/eModbus/Smartdose) type of devices is supported. 
Other WiFi sockets will need a different command or even different protocol that would have to be integrated first.
Contributions are always welcome... :wink:

The Modbus address needs to be given as described above for the first parameter of the DewAir application itself: either a DNS name or a set of IP address, port and server ID, joined by colons ':'.

#### SENSOR
Each of the two attachable sensors may be non-existent, plugged in locally into the device or other DewAir devices in the same local network.

``SENSOR`` always needs a sensor number first, that can be ``0`` for the first or ``1`` for the second DHT22 sensor.

After that we can have ``NONE`` to disable the sensor slot, ``LOCAL`` for sensors connected by wire to the device, or a Modbus address again to connect to another DewAir device.

The Modbus address has to follow the very same rules as described above for the DewAir application or the ``TARGET`` parameter.
It requires one more parameter after it, though: the sensor slot on the remote device.
This slot addresses the two sensors on that device the same way as we are doing here: ``0`` is the first, ``1`` the second sensor.

Instead of collecting the data locally, a remote sensor will be requested by Modbus. 
Note that due to the communication delays and collecting interval on the remote device the data gathered remotely may not exactly be measured at the same time the request was sent, but will be delayed for some seconds.

#### CONDITION
If and when a target gets switched is depending from the conditions you have defined in regards to the sensor data.
The ``CONDITION`` command lets you set these conditions at will.

First you will have to tell what conditions you are going to set:
- ``SENSOR 0`` or ``SENSOR 1`` will select the respective sensor
- ``DIFF`` instead will target on conditions for both sensors combined. Note that this is not possible if one or both sensors are set to ``NONE``!

The next parameter selects the data type you are using for the condition - ``TEMP`` for temperature, ``HUM`` for humidity or ``DEW`` for the dew point temperature evaluated internally.

Last is the condition proper:
- ``IGNORE`` disables it to not be regarded any more
- ``BELOW <value>`` will result ``TRUE`` if the selected data is lower than the given ``<value>``
- opposite, ``ABOVE <value>`` will be ``TRUE`` if the data is higher than ``<value>``

Example:
```
DewAir condition sensor 0 temp below 5
```
will require the first sensor's temperature to be lower than 5 degrees Celsius to be evaluated to ``TRUE``.
