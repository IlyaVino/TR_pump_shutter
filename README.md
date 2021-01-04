# TR_pump_shutter
Pump shutter for TR setup in Cuk lab

The pump shutter is designed to block the 266 pump light during closed circuit data acquisition to prevent sample burning while the cameras upload data to the computers. This will allow us to acquire more delays per row and reduces sample waste.

The program in this git has several features that allow it to be used during live tweaking and during data acquisition:
1. open/closed/triggered mode of operation
2. button press to switch between open/closed mode of operation
3. serial communication to switch mode of operation

The open/closed mode of operations are as described: shutter open or shutter closed. The triggered mode of operation is still being written but will accept an external trigger to open/close the shutter and return a 3.3 V signal to indicate the shutter status.

The button press mode of operation works when the shutter is in the open/closed mode. The idea is that the shutter can be opened and closed at the press of a button so that the user does not have to run serial operation.

The serial communication is designed for programmed opperation. The following commands are available:

sl#: sets lower/closed servo angle where # is an integer from 0 to 180 deg.

su#: sets upper/open servo angle where # is an integer form 0 to 180 deg.

m#: sets shutter operation mode where # = 0 is closed, # = 1 is open, and # = 2 is triggered

rl: return from serial lower/closed angle

ru: return from serial upper/open angle

rm: return from serial current mode of operation

fw: writes current angles to EEPROM

fr: reads current angles from EEPROM

