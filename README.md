# PID-Temperature-microcontroller

Sebastian Carter, Syed Rehman, Tahsin Sarker

[View full report on Google Drive](https://drive.google.com/file/d/1O-3gOrli30nk9of1vK4LO-9nJHyGorJx/view?usp=drive_link)

![Thermo-electric-cooler connected to a microcontroller and laptop with wires. The cooler is attached to a heatsink and fan to normalize the temperature of the opposing side.](/docs/assets/TEC-Circuit-Setup.jpg)
Hardware setup used for final demonstration.

Scope Demo Video:

[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/IsfJTwC6uQU/0.jpg)](https://www.youtube.com/watch?v=IsfJTwC6uQU)

## Abstract

Thermoelectric coolers (TEC) can be used to finely adjust temperature through heating and cooling. With the use of a microcontroller, a thermistor, and an H-Bridge, a user can set the surface of an object to maintain a certain temperature through a desktop application such as MATLAB. For the purpose of this project, the surface was the TEC module itself. Through the use of pulse width modulation (PWM) and serial communication, the magnitude and direction of heat transfer can be set from the microcontroller. Through this process, a temperature in the 0 to 40 range can be maintained with a precision of $\pm$ 0.2&deg;C. An on-board analog to digital (ADC) converter can be used to convert resistance in the thermistor to a temperature reading accurate to $\pm$ 2.7&deg;C . The method of controlling the PWM output is a PID controller with coefficients . This led to a time constant of 6.7s for a 10 timestep. This project demonstrates the versatility and efficiency of microcontrollers. The design process is much cheaper and faster with the use of a microcontroller board instead of mixed signal circuit components.

## Result

With minor overshoot, the PID controller was able to track the set temperature with an average error of ±0.2°C. 

The time constant for stepping up by 10 degrees was approximately 6.4 ± 0.5 s. 
