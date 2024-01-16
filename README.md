Using CMake to build files.

This is my project trying to read data from a gyro with a raspberry pi pico.

I have managed to get output over usb and read it with minicom

Currently cmake disables UART connection and strictly enables USB communications.


###
working notes for me:

To build in bash
/build $ cmake ..
/build $ make

To read over usb with minicom:
minicom -D /dev/ttyACM0 -b 115200
