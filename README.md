# PoliMi Sailing Team: Moth Active Flight Control System (Core Modules)

> **INTELLECTUAL PROPERTY DISCLAIMER**
> This repository contains **only the specific ROS2 modules I personally developed** for the PoliMi Sailing Team's foiling Moth. 
> To strictly protect the team's Intellectual Property and confidential hardware configurations, the full workspace (including custom message definitions, CAN bus drivers, and sensor interfaces) has been deliberately omitted. 
> **As a result, this repository is a portfolio showcase and will not compile standalone.** It is intended to demonstrate my coding architecture, control logic implementation, and ROS2 proficiency.

---

## Project Overview
As a member of the Mechatronics division of the PoliMi Sailing Team, I contributed to the development of a custom, real-time control system for a foiling Moth. The goal of the system is to process raw telemetry data, estimate the boat's state, and actively control the flight phase via custom-designed actuators.

## Full System Architecture (Context)
Although not fully published here, the complete ecosystem I worked within relies on a highly distributed and robust architecture:
* **Computing Core:** A main Raspberry Pi running a containerized **ROS2** environment via **Docker** to ensure implementation and testing.
* **Hardware Interface:** A **CAN bus** network orchestrating the communication between custom PCBs, microcontrollers, sensors and actuators.
* **Telemetry & Web App:** A secondary Raspberry Pi dedicated to WiFi telemetry, transmitting real-time data to a custom Web Application used for live monitoring and active testing functions.
To have a more general understanding of how the full system works and what are the main components this is the repository of the 2025 Software 
https://github.com/Sailing-Team-Polimi/2025_SOFTWARE.git

---

## My Contributions & Repository Structure

This repository highlights the closed-loop control algorithms and state estimation nodes I developed.

### Included Packages:
* **`flap_control`**
  * Contains the core implementation of the flight control strategy.
  * Implements a **PID controller** that fuses data from three primary sensors: a potentiometer (to measure the wand angle for height estimation), an IMU (for spatial orientation), and a GPS (for velocity tracking). Based on this telemetry, the controller computes the optimal target angle and commands a servo motor to actuate the main foil's flap mechanism, in order to make the height estimation (feedback) to follow the height target (setpoint).
* **`state_estimation`**
  * Responsible for filtering and fusing raw sensor data.
  * Provides reliable estimation of the boat's dynamic state (e.g., roll, pitch, height) despite sensor noise and vibrations.

---

## Simulation and Data Validation
Beyond the C++/Python ROS2 implementation, a significant part of my work involved:
1. **MATLAB/Simulink Modeling:** Simulating the boat's flight dynamics to pre-tune the PID controllers and validate the Kalman filter behavior before deploying the code to the physical hardware.
2. **Hardware Integration:** Assisting in the 3D design and printing of custom housings to safely fit the mechatronic hardware onboard the boat in a harsh marine environment.

