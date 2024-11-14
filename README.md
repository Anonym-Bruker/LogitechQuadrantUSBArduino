# LogitechQuadrantUSBArduino
This is for converting a Logitech/Saitek Quadrant to USB using a Arduino clone.
I did this since my quadrant used the analog din plug and I wanted to use a normal USB-port.

Hardware:
---------
Mostly followed this tutorial for the hardware:
https://forums.x-plane.org/index.php?/forums/topic/240443-saitek-throttle-quadrant-usb-conversion-using-arduino/

I also replaced the potentiometers with new ones from Aliexpress:
https://vi.aliexpress.com/item/1005004769553068.html
(I ordered the 90 degrees one. In hindsight, I should probably have considered the 120 degrees potentiometers to have more range to play with)

Used this Arduino clone (Appears in Windows as a Arduino Leonardo):
https://vi.aliexpress.com/item/32840365436.html
(The "new" USB-C 3-6V ATMEGA32U4)

Compatibility:
--------------
All buttons and axes on the throttle should work with the code.

Known Issues:
-------------
The output from the potentionmeters are not linear, so I have tried to use a exp-formula to compensate for this.
The values used might need to be changed depending on your own values.

I.e. for the Z-axis, I used "exp(0.0085*(valZ - OUTPUT_MIN_Z))*1.3" . 
As per the code, all three axis have different values here.
