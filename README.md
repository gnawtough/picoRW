Using CMake to build files.

This is my project trying to read data from a gyro with a raspberry pi pico.

I have managed to get output over usb and read it with minicom

Currently cmake disables UART connection and strictly enables USB communications.

## Hardware

Raspberry pi pico
MPU 6050 Gyro
AS5600 Magnetic Encoder -- https://www.amazon.com/Teyleten-Robot-Precision-Induction-Measurement/dp/B09LMB3PTZ


## WIRING
TODO: make wiring diagram...
both AS5600 and 6050 communicate on same bus, connect wires to same pins. 
SDAs together pin6(GP4) bus
SCLs together pin7(gp5) bus
GND pin 38
VCC pin 36

###
working notes for me:

To build in bash
/build $ cmake ..
/build $ make

To read over usb with minicom:
minicom -D /dev/ttyACM0 -b 115200
