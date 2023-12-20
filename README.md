# 3-axis-Rate-Table
A three axis rate table to simulate aircraft motion in real time.

## Overview

This repository contains the code and instructions for controlling motors and simulating flight dynamics for an aircraft using Arduino, Simulink, and an onboard ESP-32 for IMU data.

## Setup Instructions

### Step 1: Connect Motor Controllers to Power Supply

- Connect Motor Controller 1 and Motor Controller 2 to the power supply.
- Ensure proper connections and power requirements are met.

### Step 2: Run Inner Motors Code and Base Motor Code

- Run the inner motors code on Motor Controller 1.
- Run the base motor code on Motor Controller 2.
- Modify PID, voltage limits, and other values as per the specific requirements and load.
- Load the Arduino codes onto the respective motor controllers.

### Step 3: Run Simulink File and Connect Inputs

- Run the Simulink file.
- Connect inputs for roll, pitch, and yaw to the Simulink model.

### Step 4: Modify Flight Dynamic Model

- Modify the Flight Dynamic Model in the Simulink file.
- Translate inputs to aircraft orientation values.
- Ensure that the orientation values are sent via serial to the motor controllers.
- Verify that the data is also sent to Flight Gear for simulation.

### Step 5: Run Simulink Program

- Run the Simulink program with the modified Flight Dynamic Model.
- Verify that the simulated aircraft responds appropriately to the inputs.

### Additional Step (if IMU Values are Required):

- Run the `imuudpdata` code on the onboard ESP-32.
- Configure the Simulink UDP block to receive IMU values via UDP.
- Verify that IMU data is successfully received and integrated into the simulation.

