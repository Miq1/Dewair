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

#### Configuration mode

<img src=https://github.com/Miq1/DewAir/blob/master/DewAir_basic.png alt="Basic data">
<img src=https://github.com/Miq1/DewAir/blob/master/DewAir_target.png alt="Target configuration">
<img src=https://github.com/Miq1/DewAir/blob/master/DewAir_sensor.png alt="Sensors configuration">
<img src=https://github.com/Miq1/DewAir/blob/master/DewAir_combo.png alt="Combo conditions">


#### LED diagnosis

#### Button functions

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