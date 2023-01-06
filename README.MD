## Multi-purpose environment sensor device

DewAir was originally created to measure temperatures, humidity and dew point inside and outside a room and 
control a ventilator to dry out the room. 
It has been modified in the meantime to accomodate other functions as temperature alarm, remote weather sensor etc.

For this purpose it can be equipped with none, one or two DHT22 sensors, either plugged into the device or detached by a 3-wire cable.
Control over the vent is provided by a relais connected locally or over Modbus TCP to remotely switch a socket.

The device can be configured with a built-in HTML page or over Modbus TCP.
The latter also allows for data retrieval and maintenance. 
If more than one DewAir device is used, one can retrieve the sensor data from another over Modbus TCP, thus saving sensors. 

Power is supplied by USB or 5V, sensors and relais driver are powered internally.
Power consumption is below 90mA normally.

For schematics, PCB designs and 3D printable enclosures see the "Extras" folder.

### Installation

### Usage instructions

#### Start sequence
If powered on, the device will execute some initial steps until reaching normal function.
If it never was configured before (no valid configuration data found) it will immediately start Configuration mode (see next section). 
This is indicated by a slow triple blink pattern of the status LED.

Else there is a short (3s) fast blinking period. If you will press the button within this time, Configuration mode will be entered unconditionally.

Next configuration data is read, a WiFi connection is attempted with the given SSID and password. A fastly blinking LED will signal it.
Once a connection is established, both sensors are checked. If a sensor is found reacting the respective LED will turn on.

At this point the regular state is reached.

#### Configuration mode
In Configuration mode, the device will span its own WiFi network with a name of "DewAir-xxxxxx" and the given password in the source. This by default is "Maelstrom".

You will have to connect to that network to get access to the configuration. Please note that this WiFi is unencrypted and should not be kept online for too long.

Next you will want to open a browser to call the configuration HTML page with URL ``http://192.168.4.1/``. Note again that this is without TLS, so use ``http://``, not "https"!

The configuration page has several parts, described top to bottom below. Most input fields have minimum and maximum values that you may see if hovering over it with your mouse pointer.
The configuration may be invisible in parts at the first time, necessary detail input fields will be swapped in or out depending on your settings.

<img src=https://github.com/Miq1/DewAir/blob/master/DewAir_basic.png alt="Basic data">
The most important part is the first, where you will have to enter a name of the new device and the WiFi credentials of your local WiFi for the device to connect to.
Below that you may give an OTA password in case you are planning to update the firmware over air.

The next is the "master switch". This is used to completely switch the logic and target switching on or off. With it you may temporarily disable the device's activities for re-configuration etc. It is advisable to leave it OFF for the first time you are configuring it to be able to cross-check your configurations before going live.

Next comes the "fallback policy". This is to set a default target state in case the sensors or logic will fail at some time.
If set to ON, a failure will drive the device to unconditionally switch the target to ON.
The device logic may be resumed as soon as the causing failure is gone again.
The advisable state of the fallback switch is totally up to you and your application. A frost protector f.i. may better be switched ON, while a motorized door may have to be OFF instead.

Finally the "hysteresis" can be adjusted. Hysteresis means defining a number of necessary, consecutive measurements, whose results have lead to the same logic decision to switch the target.
Hysteresis consists of two components: a number of measurements in a row necessary and the time between measuring actions.
To give an example: a number of 5 measurements and a time of 20s will require the logic to decide the same for all 5 checks in 100s.
Only if all 5 resulted in either all ON or all OFF decisions, the switch finally will be done, if applicable.
The intended result is a less "nervous", steady switching behaviour that does not react on single dropout measurements.

<img src=https://github.com/Miq1/DewAir/blob/master/DewAir_target.png alt="Target configuration">
The target definition is required should you want to physically switch something on or off based on the sensor data.
If the device is used as a temp/humidity sensor only, the target may be left unconfigured.
A target may be one of three variants: "ignored", if not relevant as described before, "connected", if the device's relay port is used for it, or "Modbus" for a remote "[Smartdose](https://github.com/eModbus/Smartdose)" socket addressed via Modbus.
To use the latter, the TCP IP address, Modbus port number and Modbus server ID for it are required.

<img src=https://github.com/Miq1/DewAir/blob/master/DewAir_sensor.png alt="Sensors configuration">
Sensor configuration is quite similar to that of the target device.
The "ignored" option leaves the respective sensor slot unregarded. At the same time the combo conditions (see below) will vanish, since 
these only make sense with two sensors present.

The data for a remote, Modbus-connected sensor are the same as before for the target, with one addition: sensor slot.
A remote sensor must reside in another DewAir device to be addressed. As there are two sensor slots, you will need to name the one you are referring to.

A typical application for a remote sensor is to have just one sensor outside the building and all other devices inside it referring to the values of that one sensor.

The conditions section allows to set the conditions the respective sensor must detect to enable the target to be switched on.
Temperature, humidity or dew point can be addressed separately, the results of the evaluations are combined by a logical "and".
If you f.i. want to switch the target on a temperature higher than 10&#176;C and do not care about humidity or dew point, you will want to click on "above" in the temperature line and enter a value of 10 in the data input field. 
Both humidity and dew point options need to be set to "ignore".

Conditions set for a device that later was set to "ignore" as a device will remain unconsidered.

<img src=https://github.com/Miq1/DewAir/blob/master/DewAir_combo.png alt="Combo conditions">
Likewise the combination conditions can be set. There always is a difference of the respective measurement of sensors S0 and S1 taken as the criteria for the conditions.
The settings for the conditions are identical to those with the individual sensor conditions.


#### LED decoding
There are four LEDs on the device in this order: status, S0, S1 and target. 
I recommend to use a different LED color for the status LED for better recognition.

##### Status LED
It will signal the general state the device is in. The start sequence is described above.

In manual mode the status LED will be steadily on, the target LED will show the switch state of the target.

If the target is supposed to be off as result of the evaluation of the conditions, the status LED will display a slow single blinking pattern.

Else, if the target is to be on, the status LED will send a triple blink with a pause in between.

Whenever a measurement is initiated, the status LED will turn on and off after it.

##### Sensor LEDs S0/S1
The respective sensor LED will turn on, if the sensor was configured and a measurement was successful.
A sensor not configured will be displayed by its LED being constantly off.

If a sensor was not responding, the LED will start blinking very quickly. It will resume the regular pattern as soon as the sensor responded again.

##### Target LED
For remote (Modbus) targets, the LED will behave like the sensor leds do: steady on for responding target, quick blinking when the target did not react.

A local target has no such diagnosis, but its state (on or off) will be shown by the LED being lit for ON and dark for OFF state.

#### Button functions
Besides the special interaction during startup (see above for the start sequence) the push button has three different applications.

- Quick press<br/>will start a check of the sensors and target, if any, like is done repeatedly while the device is running.
- Long press<br/>starts manual mode. In it, the sensors are ignored and further short presses will switch the target on and off in turns.
  Another long press will end manual mode and return to automatic control.
- Double click<br/>the conditions for sensor S0, S1 and the combination conditions are checked. If the respective conditions are met,
  the respective sensor LED will light. The target LED stands for the combination conditions.

### Modbus register map

| Register address | type | Contents | Write? | Remarks |
| ----------------:| ---- | -------- |:------:| ------- |
| 1       | uint   | Master switch   | YES | Disable switching logic |
| 2, 3    | float  | S0 temperature  |     | DHT22 sensor 0 |
| 4, 5    | float  | S0 humidity     |     |                |
| 6, 7    | float  | S0 dew point    |     |                |
| 8, 9    | float  | S1 temperature  |     | DHT22 sensor 1 |
| 10, 11  | float  | S1 humidity     |     |                |
| 12, 13  | float  | S1 dew point    |     |                |
| 14      | uint   | Switch state    |     |  Current target switch state |
| 15      | uint   | Restart count   |     |  Number of reboots since last init |
| 16      | uint   | Run time in minutes | |  Minutes since last reboot. Will halt at 65535. |
| 17      | uint   | S0 health       |     |  Each '1' bit stands for a successful read |
| 18      | uint   | S1 health       |     |                |
| 19      | uint   | Target health   |     |                |
| 20      | uint   | Measuring interval in seconds | YES | Must be 10 &le; n &le; 3600 |
| 21      | uint   | Hysteresis steps | YES | Must be 1 &le; n &le; 16 |
| 22      | uint   | S0 type          | YES | 0: none<br/> 1: connected locally<br/> 2: ModbusTCP source<br/> 3: reserved |
| 23, 24  | byte[4] | S0 Modbus IP address | YES | MSB first |
| 25      | uint   | S0 Modbus port number | YES | Must be &ge; 1 |
| 26      | byte[2] | MSB: S0 Modbus server ID<br/>LSB: slot number | YES | 1 &le; MSB &le; 247<br/> LSB: 0 or 1 |
| 27      | special | S0 temperature condition:<br/> bits 14,15: condition type<br/> bits 0..13: compare value | YES | *type:*<br/> 0: disregard<br/> 1: less than value<br/> 2: greater than value<br/>3: reserved<br/>*value:*<br/> ``float val = (int(value) - 2048) / 10.0;``|
| 28      | special | S0 humidity condition/value (as above) | YES | see above |
| 29      | special | S0 dew point condition/value (as above) | YES | see above |
| 30      | uint   | S1 type          | YES | see S0 type |
| 31, 32  | byte[4] | S1 Modbus IP address | YES | MSB first |
| 33      | uint   | S1 Modbus port number | YES | Must be &ge; 1 |
| 34      | byte[2] | MSB: S1 Modbus server ID<br/>LSB: slot number | YES | 1 &le; MSB &le; 247<br/> LSB: 0 or 1 |
| 35      | special | S1 temperature condition:<br/> see S0 | YES | see S0 |
| 36      | special | S1 humidity condition/value (as above) | YES | see above |
| 37      | special | S1 dew point condition/value (as above) | YES | see above |
| 38      | uint   | Target type           | YES | 0: none<br/> 1: connected locally<br/> 2: Modbus TCP target<br/> 3: reserved |
| 39, 40  | byte[4] | Target Modbus IP address | YES | MSB first |
| 41      | uint   | Target Modbus port number | YES | Must be &ge; 1 |
| 42      | uint   | Target Modbus server ID   | YES | **MSB only!**<br/> Must be 1 &le; SID &le; 247 |
| 43      | special | (S0 temperature - S1 temperature) condition type and value | YES | see S0 temperature condition and type above |
| 44      | special | (S0 humidity - S1 humidity) condition type and value | YES | see above |
| 45      | special | (S0 dew point - S1 dew point) condition type and value | YES | see above |
| 46      | uint    | nibble 0: number of combo conditions met<br/>nibble 1: number of S1 conditions met<br/>nibble 2: number of S0 conditions met<br/>nibble 3: unused |     | debug info |
| 47      | uint    | Fallback switch setting | YES | 0: OFF<br/>else: ON |
| 48 .. 63 |    | *reserved* | YES | future extension space |
| 64      | uint    | Number of event slots |    | if 0: no events available |
| 65 ..   | special | Logged events (number see register 64) |     | bits 11 .. 15: Event code<br/>bits 6 .. 10: day/hour<br/>bits 0 .. 5: month/minute |


### Applications

#### Dew point ventilation

#### Frost protection