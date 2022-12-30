## Enclosures

There is a FreeCAD design currently only for the bare logic board without the relay or power outlet etc. This will be added at a later time.

### Logic board

The case described here is meant to fit the board with the attached LED part facing backwards. Uses are temperature/humidity sensors to be requested via Modbus or a control device with a target addressed over Modbus. 

#### Parts

Please note the names of the parts are in German - which should not matter at all, I will give the necessary descriptions below.

##### Unterschale ("Box")

The Unterschale part is the box proper to hold the PCB. The LEDs ans switch are to be soldered to the back side of the PCB, opposite to all other parts.
Holes in the bottom of box will hold the 4 LEDs and button in place.

The openings in the box for sensors 0 and 1, 5V power connector, relay connector and the MCU's USB port are configurable as are required in the FreeCAD file provided.
There is a DynamicData object called "Masse" in the construction, where a "Hasxxx" boolean variable is provided for each opening.
If set to ``true``, the opening will be activated.

<img src=https://github.com/Miq1/DewAir/blob/master/Extras/Enclosure/SelectOpening.png alt="Openings selection">