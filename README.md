# ELEC5552 - 2025 - Team-2-08
Open-source Indoor Surveillance Drone by UWA Team 2-08 for ELEC5552. Includes ESP32 flight-controller firmware, Python ground-station code, and Altium PCB files for custom FC and 4-in-1 ESC using AM32. Designed for safe indoor flight, testing, and future open-source development.

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

<img width="977" height="513" alt="image" src="https://github.com/user-attachments/assets/0c17ce76-736a-4fa9-a08f-7abcbf10ef4e" />

Power and ground plane separation within the ESC PCB stack-up.
