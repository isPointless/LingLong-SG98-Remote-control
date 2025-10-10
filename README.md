# LingLong SG98 Remote control
A remote control PCB & software to control a grinder with a servo motor.
Currently being built for Linglong SG98 using RT and JMC servo drive.

#### How to use
Aquire a PCB (order through JLCPCB with the provided files or buy a readymade one). and wire it up correctly.
Install Visual Studio code, install the PlatformIO extension. Clone this repo.

Make sure to (un)comment the proper drive type in definitions.
Uploading to ESP32 through USB-C. Make sure +24v is not present. If necessary short press EN while holding BOOT0 button for uploading.
If flash error in final install, plugging in the USB C shortly after pressing "Upload" will make it work.

#### BOM
- ESP32 PCB board + 90 degree 2.54 pins
- 1.69" round corner SPI color display + 2.54 female header
- 19mm ultra short throw momentary button with 3-5V LED.
- M12 4 pin female connector
- KCD1 SPST switch
- 3D printed box, including 4xM3x4Wx5L heat inserts and 2x m3*8 countersunk screw and m3*6 non-countersunk screw.
- A knob for the rotary encoder to your choosing.

- 
