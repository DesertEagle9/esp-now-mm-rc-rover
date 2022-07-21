# esp-now-mm-rc-rover
Arduino code to implement wireless control of a [Micromelon Rover](https://micromelon.com.au/) using multiple ESP32 remote controllers. The two files in this repository are:

**1. ESP-NOW-remote-control-sender.ino:**
This is the source code for the ESP32 remote controller. The code enables the ESP32 remote controller with pushbuttons (see wiring diagrams) to wirelessly transmit movement commands via ESP-NOW to the ESP32 receiver which relays the received commands to a Micromelon Rover via UART.


**2. ESP-NOW-remote-control-receiver.ino:**
This is the source code for the ESP32 receiver which is connected to the Micromelon Rover via UART (see wiring diagrams). This code enables the ESP32 receiver to receive commands from multiple ESP32 remote controllers via ESP-NOW to the Micromelon Rover for handling and exectution. 

**Useful Links:**
* [Rui Santos ESP-NOW tutorial](https://RandomNerdTutorials.com/esp-now-esp32-arduino-ide/)
* [Micromelon Python API](https://github.com/Micromelon-Robotics/mm-pymodule)
* [ESP32 TTGO T7 V1.3 Mini32](https://www.aliexpress.com/item/32846710180.html?_randl_currency=AUD&_randl_shipto=AU&src=google&src=google&albch=rmkt&acnt=576-373-4425&albcp=16560254345&albag=&slnk=&trgt=&plac=&crea=&netw=x&device=c&mtctp=&albbt=Google_7_rmkt&gclid=CjwKCAjwy_aUBhACEiwA2IHHQCae2f6CnmNbD0QBvAhtqfIuzd_kipWZuzskXRm6EVVpVPYGZNCiKBoCiBgQAvD_BwE&aff_fcid=ed62039dbced430782711f9484dc9d71-1654506581935-06456-UneMJZVf&aff_fsk=UneMJZVf&aff_platform=aaf&sk=UneMJZVf&aff_trace_key=ed62039dbced430782711f9484dc9d71-1654506581935-06456-UneMJZVf&terminal_id=60c76bfabdf64f9db3a17ac94d1f5248&afSmartRedirect=y)
