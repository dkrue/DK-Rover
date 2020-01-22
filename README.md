# DK-Rover
DK-Rover is an [ESP32](https://www.adafruit.com/product/3405) based mobile robot project that responds to commands via wifi. This is an Arduino project I tinker with when I feel like driving it around the yard or upsetting the cat.

![The DK-Rover Robot](/images/DK-Rover_top.jpg)
## Features
### Dual-mode wifi connection
Connect your phone to the rover's access point broadcast on its own wifi network, or connect through a local router. Going through a router works great indoors, but offers very limited range outside. It can do both simultaneously. 
### Web based UI served from the rover
This project uses the SPI file system to serve an html page from flash memory to control the rover.
### Cool stuff
My only goal with this project is to pack as much cool stuff as I can onto the rover. Don't forget to toggle the headlights!
#### Current sensors
- Ultrasonic range sensor (HC-SR04, 3.3v version)
- Dual infrared corner sensors (great indoors, useless outside)
- Tilt sensor (stops the rover if it's about to flip over)
- Li-ion battery voltage (still fine tuning the algorithm on this)
#### Future sensors
- VGA Camera (a cheap QiFei OV7670)
- Barometric, gas, temperature, humidity sensor ([Bosch BME680](https://www.adafruit.com/product/3660))
## Hardware
- [Adafruit HUZZAH32 – ESP32 Feather Board](https://www.adafruit.com/product/3405)
- [Adafruit DC Motor + Stepper FeatherWing](https://www.adafruit.com/product/2927) H-Bridge motor controller
- Adafruit Stackable 160x80 Color LCD
- [4WD Car Chassis + DC Motors](https://github.com/SmartArduino/XPT/blob/master/SR11.pdf)
- 2x 18650 li-ion batteries for drive circuit (11,000mAh each)
- 3.7v li-ion battery for logic circuit
## Improvements
Currently everything is on a breadboard. Once I get the other sensors nailed down I'll move everything over to soldered protoboard.
It would also be great to have some sort of waterproofing so I could drive it around in the snow.
## Resources
[ESPAsyncWebServer on GitHub](https://github.com/me-no-dev/ESPAsyncWebServer)

[ESP32 Web Server using SPIFFS (SPI Flash File System)](https://randomnerdtutorials.com/esp32-web-server-spiffs-spi-flash-file-system/)
