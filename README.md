# ArduiSNES
Four SNES controllers to USB using Arduino Leonardo board

# Project configuration and compilation
Clone the project locally, then in a shell terminal type the following commands

```
git submodule init
git submodule update
make
```

To flash the Arduino Leonardo, plug it and push the reset button, then type 
```
make avrdude
```

**Note:** If you are on Windows or if you already have other devices named "/dev/ttyACMx", 
you need to edit the makefile and change the "AVRDUDE_PORT" variable so it matches the right device.

I don't know if it is a design issue but it seems like flashing the Arduino Leonardo can be a bit tricky sometimes
and requires some timing. 
So, if you have any errors during the flashing process, unplug/replug the board, 
try again or look on the internet for additional information.
