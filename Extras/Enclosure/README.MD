## Enclosures

There is a FreeCAD design currently only for the bare logic board without the relay or power outlet etc. This will be added at a later time.

### Logic board ("DewAir_klein_V2")

The case described here is meant to fit the board with the attached LED part facing backwards. Uses are temperature/humidity sensors to be requested via Modbus or a control device with a target addressed over Modbus. 

#### Parts

Please note the names of the parts are in German - which should not matter at all, I will give the necessary descriptions below.
Also please refer to the picture below!

<img src=https://github.com/Miq1/DewAir/blob/master/Extras/Enclosure/DewAirCAD.png alt="Parts overview">

##### Unterschale ("Box")

The Unterschale part is the box proper to hold the PCB. The LEDs and switch are to be soldered to the back side of the PCB, opposite to all other parts.
Holes in the bottom of box will hold the 4 LEDs and button in place.

The openings in the box for sensors 0 and 1, 5V power connector, relay connector and the MCU's USB port are configurable as are required in the FreeCAD file provided.
There is a DynamicData object called "Masse" in the construction, where a "Hasxxx" boolean variable is provided for each opening.
If set to ``true``, the opening will be activated.

<img src=https://github.com/Miq1/DewAir/blob/master/Extras/Enclosure/SelectOpenings.png alt="Openings selection">

Note there are two ``HasUSB...`` variables that each will match the type of MCU module you are using - NodeMCU or Wemos D1 mini.

##### Deckel ("Lid")

The Deckel part is the closing lid of the box. It will be secured by 4 M3 screws on the corners to the Unterschale part.
Note that there are two STL files for this part. The one called ``...Deckel_Hang.stl`` contains a lid with four fastening tabs attached. 
In the FreeCAD file, these can be activated by setting the ``HasHangers`` boolean variable to ``true``.

##### Taste ("Button")

The Taste part is set into the largest bottom hole in the box and will allow to click the button on the PCB from outside of the box.
It is advisable to print it in a different color than the box to have it stand out.

##### LED-Riser

Since the necessary distance between the PCB and the box needs to be larger than the LEDs' lengths to fit the switch button, LEDs need to be elevated a bit from the PCB when soldered.

The LED-Riser part need to be printed and put in between the LEDs and the PCB before soldering these.

#### Printing

I would recommend to use PETG or ABS for the parts for better resilience of the case. Depending on the environment a PLA case may suffer from UV radiation or wet conditions.

The LED-Riser part needs to be printed first, for it will have to be used when soldering the PCB already.

#### Assembly

Depending on the environment where the device is deployed up to all 8 PCB sockets can be equipped with melt-in threads for maximum stability. 
The minimum would be the 4 sockets at the 4 corners, but I would advise to have the two additional sockets for the LED and button part used as well to prevent the board breaking if too hard a pressure is applied to the button.

First set the Taste part into the matching hole of the box, then put in the PCB, gently guiding the LEDs into their holes.
Fasten the PCB with M3 screws to the sockets.

Insert 4 more melt-in threads into the corner blocks of the Deckel part. Put on the lid and secure it with 4 M3 countersunk screws to the box. The screw heads shall be completely sunk into the box' holes.

### DHT22 housing

This is a little "bird cage" case for the DHT22 temperature/humidity sensors to be attached outside. 
The bottom part may be bolted to a wall, then the DHT22 with cable is laid into it and finally a lid is fastened to it.
Thus you may open the case and take out the sensor without having to unscrew the case from the wall again.
The cable is supposed to go through the S-shaped channel at the bottom to prevent it from being ripped off.
The case has air vent openings facing down to prevent rain coming into the case.

<img src=https://github.com/Miq1/DewAir/blob/master/Extras/Enclosure/Sensor.jpg alt="Sensor case">