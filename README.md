# mypool controler
Pool automation and metering.

Still in development: It means, I am in middle of testing phase. You can see my scatches for PH and ORP meassuring 
witch both of them includes 2points calibration process and new one based on ADS1015. 

Dosing; PH- and PH+ aditing to the water and solar heating, lights automation ... and mobile app (nased on Blynk) control.

PH/ORP measuring based on the same cheap analog module (and the new based on ADS1015 12bits precise measuring):

http://www.ebay.com/itm/Liquid-PH0-14-Value-Detect-Sensor-Module-PH-Electrode-Probe-BNC-for-Arduino-/201758085638?hash=item2ef9b81206:g:oTUAAOSwLOtYXCqf
event. https://www.ebay.com/sch/i.html?_from=R40&_sacat=0&_nkw=ads1015&_sop=15

For ORP you have to cut off first trimmer (clouse to BNC) and replace it with 30k Ohm resistor. Thats all.

Note: This module needs very stable voltage on his input. In case of you will use both module PH/ORP in same water, I am recomande to add to the front of your module galvanic separation, like NME0505SC, in case you desided to use ADS1015 for both modules (pH/Rx) is higly recomanfed to use opto-divider for both ADS1015, something like ADUM1251. Benefit - ground-loops free.
