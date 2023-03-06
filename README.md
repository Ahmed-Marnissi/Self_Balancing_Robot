# Self_Balancing_Robot
This project implements a self-balancing robot using an Arduino microcontroller and an MPU6050 sensor. The robot is designed to maintain balance while moving on two wheels and can be controlled through a remote control.

Table of Contents
Hardware Requirements
Software Requirements
Installation
Usage
Contributing
License
Hardware Requirements
Arduino Uno or similar microcontroller
MPU6050 sensor
Motor driver module (L298N)
DC motors (2)
Wheel encoders (2)
Wheels (2)
Battery pack (6-12V)
Remote control (optional)
Software Requirements
Arduino IDE
Libraries:
Wire
PID
MPU6050
Encoder
Installation
Connect the components according to the circuit diagram in the "circuit-diagram.png" file.
Upload the "self_balancing_robot.ino" file to the Arduino board using the Arduino IDE.
Usage
Turn on the power supply for the robot.
Wait for the MPU6050 sensor to initialize (the LED on the Arduino board will stop blinking).
Place the robot on a flat surface and it should balance itself.
Use a remote control to move the robot forward, backward, left, and right.
Contributing
Contributions to this project are always welcome. To contribute, please fork this repository, make your changes, and submit a pull request.

License
This project is licensed under the MIT License - see the LICENSE.md file for details.