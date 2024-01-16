This is my project trying to read data from a gyro with a raspberry pi pico.

I have managed to get output over usb and read it with minicom



###
working notes for me:

To build in bash
/build $ cmake ..
/build $ make

To read over usb with minicom:
minicom -D /dev/ttyACM0 -b 115200
