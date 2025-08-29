# RCSkyhawkTrainer
Building and coding a medium sized RC 172 Cessna Skyhawk Model (Tailwheel Mod) from scratch using an Arduino and Foamboard.

## Project Process
1. Plan everything (Materials, blueprint, wiring)
2. Build aircraft
3. Aircraft glide test
4. Put electronics together
5. Write and test code (many iterations)
6. Review weaknesses and improvements
7. Get License for flying
8. Fly plane

## What I Learned
- Develop simple blueprints/sketches
- Solder and solder maintainance
- Improvise in design and code
- Electronic Speed Controllers, Battery Eliminator Circuits, Arduino/Microcontrollers, Digital & Analog & PWM Signals, Radio Transmission (Sending and Recieving) on NRF24l01, Serial Communication and Data Structures, Custom Circuits, Writing Failsafes in Code, Watchdog Timers, Madgwick and Kalman Filter, DMP6 (Digital Motion Processor) on MPU6050, PID Controllers

## 3 Main Code Components

Python (Main)
- Recieve telemetry data
- Send plane commands/controls
- Visual Dashboard
- Xbox Controller for Plane (Pygame)

Arduino (Computer)
- Relay data between computer and plane via NRF24L01

Arduino (Plane)
- Recieve plane commands to execute
- Send back telemetry data like gyroscope and battery voltage
- PID controller for autostabilization via gyroscope

## Materials Used: 
- Foamboard
- Paper clips
- Bamboo skewers
- Popsicle sticks
- Plastic Straws

## Electrical Parts Used:
- 2 Ardunio Uno
- Elegoo Protoshield v5
- Mini breadboard
- Xbox Controller (wired)
- MPU6050 6 axis
- 4 SG90 Servos
- 1400 KV A2212 Motor + 40A ESC
- 5V 5A UBEC
- 2200mah 11.1V 3S Lipo battery
- 2 NRF24L01+PA+LNA, 100uf 16V capacitors, resistors (100k, 10k)

## Media:
- Plane Blueprint: In Repository
- Wiring Diagram: In Repository

## Resources:
- DMP6 Code example for Electronic Cats MPU6050: https://github.com/ElectronicCats/mpu6050/blob/master/examples/MPU6050_DMP6/MPU6050_DMP6.ino
- Foam Wing: https://www.youtube.com/watch?v=s-btyW70FOU
- NRF24L01 Wiring: https://howtomechatronics.com/tutorials/arduino/arduino-wireless-communication-nrf24l01-tutorial/
- MPU6050 Wiring: https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/
- Good Linkage Geometry for Control Surfaces: https://youtu.be/U3AHgk0SUoI?si=rI_dNsmJctV9-yjQ
- Cessna CAD Model: https://cad.onshape.com/documents/fbee1c690bef929a04a1c229/w/799ae59c6a1b1904c36a38a1/e/53bef2ebf86cd061f18f0ac2?renderMode=0&uiState=68a7b4c42b0a2405962bdc53
