# ELEC5552 - 2025 - Team-2-08
Open-source Indoor Surveillance Drone by UWA Team 2-08 for ELEC5552. Includes hardware, firmware and software. Designed for safe indoor flight, testing, and future open-source development.

# FC
## Specifications
Two-layer custom Flight Controller PCB designed in Altium. Integrates ESP32-WROOM-32E microcontroller, ICM-42670P IMU, ultrasonic and IR sensor interfaces, and a 10-pin JST connector for ESC communication. Operates on 3.3 V logic and 5 V sensor input, with regulated power, compact form factor, and mounting compatibility with the drone chassis.
## Images
<img width="295" height="378" alt="image" src="https://github.com/user-attachments/assets/3bbbf2b7-f200-44a1-afea-a521d399d14a" />

Top layer view of the Flight Controller PCB showing component layout and routing.

<img width="294" height="378" alt="image" src="https://github.com/user-attachments/assets/1a4abd39-1b68-45d0-9570-f096fdec3646" />

Bottom layer (ground plane) view showing grounding and power distribution.

<img width="283" height="377" alt="image" src="https://github.com/user-attachments/assets/6ba4aa8b-8443-49db-8b34-9635e545ef8a" />

3D rendered view of the completed Flight Controller PCB.

# ESC
## Specifications
Four-layer 4-in-1 Electronic Speed Controller (ESC) PCB integrating buck-boost and 3.3 V regulation stages, current-sense amplifier, STM32 microcontrollers, gate-driver ICs, and low R<sub>DS(on)</sub> MOSFETs for efficient BLDC motor control. Compatible with 1 S 3.7 V 800 mAh Li-Po battery and AM32 firmware. Designed for compactness, low noise, and safe integration within the quadcopter chassis.
## Images
<img width="974" height="498" alt="image" src="https://github.com/user-attachments/assets/4aafa16f-e6b7-4d61-948f-2823ab3746d8" />

Realistic top and bottom view of the ESC PCB showing component placement and overall layout.

<img width="1071" height="1052" alt="image" src="https://github.com/user-attachments/assets/39927c7c-3619-46ff-9020-6a75a05f0218" />

Top, power, ground, and bottom plane views showing component layout and routing.

# Software
## Flight Controller PCB
The flight controller PCB, based on an ESP32 microcontroller with an inbuilt IMU and support for an attached time of flight, radar distance and optical flow sensor.
```
~/FC PCB - Altium Files/
```
The schematic and PCB were designed using Altium, and sent to PCBWay for maunfacturing.
## Electronic Speed Controller PCB
The ESC PCB, which is a four-in-one designed to power all motors of the quadcopter and regulate/distribute power to the flight controller and camera modules.
```
~/ESC PCB - Altium Files/
```
The motor channels are powered by STMicroelectronics MCUs and are intended to run AM32 ESC firmware.
## Flight Controller Firmware - Custom Version (retired)
The original flight controller firmware we intended to use, written in ESP-IDF for an ESP32 using FreeRTOS for scheduling.
```
~/FC Firmware - Version 1/
```
Communication over Wi-Fi using Websockets for data transfer. Complete with a web-based interface for piloting, adjusting PID parameters 'live' without re-writing firmware, and manually overriding setpoints and outputs.
This version was retired as the performance was not adequate for our liking.
## Flight Controller Firmware - Crazyflie Version
A revamped firmware package for our flight controller, this time based on the Crazyflie platform firmware ported to ESP32.
```
~/FC Firmware - Version 2/
```
This package was selected as it had pre-bundled drivers for our peripherals and a much more advanced arsenal of control loops, predictors and situational awareness handlers. 
Many functions were modified or replaced with custom versions to support our hardware and use case. Some of the larger changes include:
* The ESC communication is custom, and was written to translate Crazyflie's motor ratio (a value between 10000 and 60000) into a signal that BLHeli ESCs understand (pulse between 1000ms and 2000ms).
* The time-of-flight sensor drivers were modified to support the initialisation of a second device and address change of one of the devices so they can operate on the same bus simultaneously. 
## Ground Station Python Scripts
These files handle the autonomous navigation step-through, manual control takeover, and video recording via a laptop connected to the drone's Wi-Fi.
### Flight Planner and Manual Control
```
~/Ground Station/planner_interface.py
```
This script allows the planning of a flight path via mouse inputs, clicking points on a grid to set the x,y coordinates for the drone. It communicates these sequentially to the drone. There are also buttons to manually pilot the drone, and cut power when required.
### Video Recording
```
~/Ground Station/cam_handler.py
```
This script, upon launch, opens a stream from the ESP-CAM over the network and encodes it into a .avi file.
