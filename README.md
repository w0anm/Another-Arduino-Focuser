# Another-Arduino-Focuser
An inexpensive Arduino  focuser for Ekos/INDI

This focuser emulates a "SmartFocuser" for use with Ekos/Indi.  

Select "Smart Focuser" for the focuser driver in indi.  You will need to setup the 
steps limits of 1638 and the usb serial device.  Ekos driver does not save these values.

Current configuration for limits is 0 through 1628 and mid range default is 819 steps.
