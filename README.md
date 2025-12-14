# final_project
PID Line Follower Robot
# Advanced PID Line Follower Robot  
### MSP432 – ECE 528 Final Project  
Author: ziyi zhao  
Date: December 2025  

---

## Introduction  
This project implements an **advanced PID-controlled autonomous line follower robot** using the TI MSP432 LaunchPad.  
The robot follows a black-line track with curves, S-turns, and 90° corners.  
The control system integrates:

- PID feedback control  
- Adaptive speed adjustment  
- A finite state machine for handling sharp turns  
- Reflectance sensor–based error computation  
- PWM motor control  
- A custom-designed test track  

The robot successfully completes a full loop on the designed track.  
Straight-line oscillation is still present, indicating further PID tuning can improve performance.

---
## Functional Block Diagram

<img width="578" height="643" alt="image" src="https://github.com/user-attachments/assets/ccabcbc1-59c2-4ff4-aa77-f20a98ae8d5a" />

## Results and Video Demonstration  


- Demo Video:https://youtube.com/shorts/zgAQ4hcRCAM?si=lXNsswOfacVpzPVK
 

The robot demonstrates stable tracking, smooth turns, and robust handling of 90° corners and T-junctions.

---

## Background and Methodology  

### Embedded Systems Concepts Applied  
This project uses several key embedded systems techniques:

- **Timer interrupts (Timer A1)** for deterministic 10 ms control loop  
- **PWM generation (Timer A0)** for motor speed control  
- **UART communication** (Bluetooth tuning design)  
- **GPIO sensor interfacing** with an 8-channel reflectance array  
- **Finite State Machines** to handle discontinuous control scenarios  
- **Closed-loop feedback control** using PID  

These concepts together form a complete real-time embedded control system.

---

###  How the Project Goals Were Achieved  
This project implements an advanced PID-based autonomous line follower robot using the TI MSP432 microcontroller. The robot is designed to follow a black-line track containing straight segments, S-curves, quarter-circle arcs, and 90° turns. A reflectance sensor array is used to detect the line position, and the middle five sensors are processed with weighted averaging to compute a smooth and symmetric tracking error. A PID controller uses this error to generate a steering correction, while adaptive speed control automatically reduces speed in curves and increases it on straight lines, improving stability and preventing overshoot. For handling discontinuous situations such as sharp turns, T-junctions, and dead ends, a finite state machine supplements PID control with deterministic turning behaviors. The control loop runs every 10 ms using Timer A1, and PWM motor control is achieved through Timer A0. The robot successfully completes a full loop on a custom-built track, demonstrating smooth tracking, reliable turning, and robust autonomous behavior. Although straight-line oscillation is still present, indicating further tuning opportunities for Kp, Kd, and speed scaling, the system effectively showcases embedded systems concepts including timers, interrupts, PWM generation, UART communication, sensor interfacing, and hybrid control architecture.

## Table of Pinout Used
| MSP432 Pin | Function | Connected To |
|------------|----------|--------------|
| P7.0–P7.7 | Reflectance sensor inputs | QTR sensors |
| P2.4 (TA0.1) | PWM Left Motor | Motor driver A-IN1 |
| P2.5 (TA0.2) | PWM Right Motor | Motor driver B-IN1 |
| P5.4, P5.5 | Motor direction | Motor driver DIR pins |
| P3.6, P3.7 | Motor enable | Motor driver EN pins |
| P9.6 (UCA3RXD) | UART RX | BLE TX *(optional)* |
| P9.7 (UCA3TXD) | UART TX | BLE RX *(optional)* 

