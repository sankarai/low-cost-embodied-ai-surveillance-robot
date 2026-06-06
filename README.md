# low-cost-embodied-ai-surveillance-robot

Low-cost embodied AI robot for autonomous indoor surveillance using ESP32, IMU-based navigation, and YOLO person detection.

## Overview

This project demonstrates a low-cost embodied AI robot for indoor surveillance, autonomous patrol, obstacle recovery, multiview scanning, and real-time person detection. The platform combines an ESP32-based mobile robot, onboard sensing, wireless video streaming, and Python-based computer vision processing to investigate practical embodied AI concepts using affordable hardware.

## Key Features

* ESP32-based robot control
* OV3660 camera streaming
* Ultrasonic obstacle sensing
* Retrofitted MPU6050 IMU for yaw-based heading alignment
* Python-based patrol controller
* YOLO-based person detection
* Multiview camera scanning
* Autonomous obstacle recovery

## Attribution

This project is based on the Freenove 4WD Smart Car Kit (FNK0053). Several ESP32 firmware components are derived from or based on software supplied with the Freenove platform and remain subject to the original Freenove licence terms.

Additional modifications developed as part of this project include:

* MPU6050 IMU integration
* Yaw estimation and heading alignment
* Autonomous patrol behaviour
* Multiview environmental scanning
* External Python control platform
* Computer vision-based person detection
* Patrol coverage estimation and recovery strategies

## Licensing

This repository contains a mixture of original and third-party software components.

* Freenove-derived firmware components remain subject to the original Freenove licence terms.
* Python-based patrol, navigation, and computer vision software was developed as part of this project.
* Users are responsible for complying with the licence terms of all third-party libraries and frameworks used by this project.

## Repository Status

This repository is provided for educational and research purposes. Users should review the licence terms associated with Freenove and other third-party components before redistribution or commercial use.
