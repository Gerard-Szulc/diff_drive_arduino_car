# diff_drive_arduino_car
Arduino car controlled by serial json commands with joystick differential drive mappng.
Communication and steering comes from serial json commands using ArduinoJson library.

Requires platformio platform, Arduino and ArduinoJson libraries.

Personally using esp32 connected with DS3 controller connected by tx/rx pins. 

Commands for buttons are structured:
```
{
  "button": int
}
```
 using analog stick: 
 ```
{
  "x": int,
  "y": int,
}
```
button mapping:
```
const int BT_STOP = 10;
const int BT_FWD = 1;
const int BT_BCK = 2;
const int BT_RIGHT = 3;
const int BT_LEFT = 4;
const int BT_MODE = 9;
```

BT_MODE is used to switch between manual and autonomous mode.

Steering algorithm thanks to Calvin Hass from https://www.impulseadventure.com/elec/robot-differential-steering.html .
