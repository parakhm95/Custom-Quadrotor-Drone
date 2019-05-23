Libraries required :
--------

https://github.com/TMRh20/RF24

https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/I2Cdev


Hardware used :
---
- Arduino Nano / ATmega328
- MPU - 6050
- BMP180
- nRF24L01 PA LNA
- A2212 1000kV motors with 1045 Props


PID Sets for smooth or sharp control:
-------

##### SET A - Smooth handling and no jitters.

P(Pitch & Roll) - 1.3
I(Pitch & Roll) - 0.0105
D(Pitch & Roll) - 68.4

P(Yaw) - 4.0
I(Yaw) - 0.00525


##### SET B - Sharp Handling and very tiny jitters(being resolved).


P(Pitch & Roll) - 2.5
I(Pitch & Roll) - 0.0105
D(Pitch & Roll) - 63.4

P(Yaw) - 4.0
I(Yaw) - 0.00525



