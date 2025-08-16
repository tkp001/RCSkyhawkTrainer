# RCSkyhawkTrainer
Building and coding a medium sized RC 172 Cessna Skyhawk Model (Tailwheel Mod) from scratch using an Arduino.

**3 Main Components:**

Python
- Visual Dashboard
- Xbox Controller for Plane

Arduino (Computer)
- Relay data between computer and plane via NRF24L01

Arduino (Plane)
- Recieve plane commands to execute
- Send back telemetry data like gyroscope and battery voltage


Plane Blueprint: In Repository

Wiring Diagram
- to be added

Parts Used:
- 2 Ardunio Uno
- MPU6050 6 axis
- 4 SG90 Servos
- 1400 KV A2212 Motor + 40A ESC
- 5A 5A UBEC
- 2200mah 11.1V 3S Lipo battery
- 2 NRF24L01+PA+LNA, 100uf 16V capacitors, resistors (100k, 10k)
