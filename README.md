Standup Timer - PIC16F1455
==========================================

## What is it?
Along with my [Standup Timer app](https://github.com/joeostrander/StandupTimer), monitor your sit/stand time.  
If sitting too long, the app warns to stand up!  There is Arduino code too.


## Requirements
* StandupTimer app (C#)
* Switch (like a magnetic reed style or other)
* PIC16F1455

## Wiring
* USB D+ (green wire) to pin RA0
* USB D- (white wire) to pin RA1
* USB GND (black wire) to pin GND (Vss - pin 14)
* RC5 (Rx) to Tx of serial destination device (optionally via a 220 Ohm resistor)
* RC4 (Tx) to Rx of serial destination device (optionally via a 220 Ohm resistor)
* A 0.1µF cap between Vdd (pin 1) and GND
* A 0.1 to 0.47µF cap between Vusb (pin 11) and GND
* RC3 (pin 7) to LED+Resistor
* RC2 (pin 8) and GND to reed switch


## Credits
* http://wordpress.codewrite.co.uk/pic/2013/07/26/usb-interface-using-pic16f1455-2/
* https://github.com/jgeisler0303/PIC16F1454_USB2Serial