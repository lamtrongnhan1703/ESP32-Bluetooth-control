# Gesture-Controlled Robot Car using ESP32

## General Description
This project focuses on the design and development of a remote-controlled robotic vehicle operated by hand gestures. Instead of using a traditional joystick, the user wears a smart glove. This glove reads the tilt angles and motion states of the hand, wirelessly transmitting these signals to the robot to control its navigation (forward, backward, turn left, turn right, and stop). 

The system operates on a Master-Slave communication architecture via Bluetooth between two ESP32 microcontrollers, ensuring low latency and high stability during real-time control.

## Technologies & Hardware
* Microcontrollers: 2 x ESP32 (Dual-core, built-in Wi-Fi/Bluetooth).
* Programming Language: C/C++.
* Development Environment (IDE): Visual Studio Code (VS Code) with PlatformIO extension.
* Communication Protocol: Bluetooth Classic / Bluetooth Low Energy (BLE).
* Sensor (Glove): MPU6050 (6-axis Accelerometer & Gyroscope) for hand tilt recognition.
* Motor Control (Car): Motor driver module (L298N / TB6612 or equivalent) and DC Motors.

## Project Structure
The project is divided into three main directories to effectively manage the source code and documentation:

### 1. /Report
This directory contains all the project documentation. It includes the theoretical background, schematics, hardware design, system algorithm flowcharts, and practical testing results.

### 2. /Glove_Transmitter
Contains the C/C++ source code flashed to the ESP32 mounted on the glove. This unit is responsible for reading gesture data from the sensor, filtering noise, and packaging the data to be sent via Bluetooth.

### 3. /Car_Receiver
Contains the C/C++ source code flashed to the ESP32 mounted on the car. This unit is responsible for receiving the Bluetooth signal from the glove, decoding it, and outputting PWM signals to drive the wheels accordingly.

## How to Run the Project
Because the system consists of two independent hardware components, the specific setup, wiring instructions, and steps to build/upload the code are separated. 

Please navigate to the respective sub-directories and read their local README files for detailed instructions:
* For the glove transmitter: Refer to the `README.md` inside the `/Glove_Transmitter` folder.
* For the car receiver: Refer to the `README.md` inside the `/Car_Receiver` folder.

