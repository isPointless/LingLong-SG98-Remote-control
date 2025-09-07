# LingLong SG98 Remote control
A remote control PCB & software to control a grinder with a servo motor.
Currently being built for Linglong SG98 using RT and JMC servo drive.

#### How to use
Aquire a PCB (order through JLCPCB with the provided files or buy a readymade one). and wire it up correctly.
Install Visual Studio code, install the PlatformIO extension. Clone this repo.

Make sure to (un)comment the proper drive type in definitions.
Uploading to ESP32 through USB-C. Make sure +24v is not present. If necessary press EN while holding BOOT0 button for uploading.

#### BOM
- ESP32 PCB board
- 1.69" round corner SPI color display
- Short throw momentary button with 3-5V LED.
- 2 and 3 pin a