# mypool controler
Pool automation and metering.

Still in development: It means. I am in middle of testing phase. You can see part of my scatch for PH and ORP meassuring 
witch both of them includes 2points calibration process. Later I will add whole scatch for pool automatization (inc. chemistry
dosing; PH- and PH+ dditing to the water) and solar heating, lights automation ... and mobile app (nased on Blynk) for full control on it.

PH/ORP measuring based on the same cheap analog module:

http://www.ebay.com/itm/Liquid-PH0-14-Value-Detect-Sensor-Module-PH-Electrode-Probe-BNC-for-Arduino-/201758085638?hash=item2ef9b81206:g:oTUAAOSwLOtYXCqf

For ORP you have to ca of first trimmer (clouse to BNC) and replace it with 30k Ohm resistor. Thats all.

Note: This module needs very stable voltage on his input. In case of you will use both module PH/ORP in same water, I am recomande to add to the front of your module galvanic separation, like NME0505SC. 
