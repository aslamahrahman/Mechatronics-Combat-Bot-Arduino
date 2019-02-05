## Mechatronic Combat Bot
- Arduino code for a combat bot designed for the final project in MEAM 510 Design of Mechatronic Systems, University of Pennsylvania
- Overview of capabilities of bot:
    - Motion on the ground, driven by a joystick. Steering achieved using differential drive with custom turning radius.
    - Attacking capabilities with a hammer mounted on two servos, enabling two degrees of freedom. Hammer also acts as a shield.
    - Health display, healing animation on a Neopixel LED ring. Communication using I2C protocol.
    - Attacking animation on LED Matrix display. Communication using SPI protocol.
    - Audio effects over DAC.
    - Autonomous motion using ultrasonic sensors.
- Code written in Arduino and C++
- More detailed description of the project can be found in the file titled 'Report'
- Circuit diagram and software architecture below

![alt text](https://github.com/aslamahrahman/Mechatronics-Combat-Bot-Arduino/blob/master/Circuit-diagram.png)
![alt_text](https://github.com/aslamahrahman/Mechatronics-Combat-Bot-Arduino/blob/master/Software-architecture.png)
