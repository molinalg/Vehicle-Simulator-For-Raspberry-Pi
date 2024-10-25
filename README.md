# Vehicle Simulator For Raspberry Pi

## Description
Developed in 2024, "Vehicle Simulator With Raspberry Pi" is a university project made during the fourth course of Computer Engineering at UC3M in collaboration with @EnriqueMorenoG88.

It was made for the subject "Foundations of Internet of Things" and corresponds to one of the practices of this course. The main goal of the project is to learn how to program code that uses **sensors** and **actuators** in **Python**. These components were part of a circuit mounted on a **breadboard** and connected to a **Raspberry Pi 3B**.

**NOTE:** Part of the code and the comments are in spanish.

This project was continued in [this one](https://github.com/molinalg/Vehicle-Simulator-Docker) and adapted to the use of **dockers, microservices, databases and the mosquitto protocol**.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Problem Proposed](#problem-proposed)
- [License](#license)
- [Contact](#contact)

## Installation
To execute this code, first execute this command to install the neccesary library:
```sh
pip install RPi.GPIO
```

A Raspberry Pi device is also needed tu run the code and connect to a breadboard with the circuit (the pins used are in the code).

## Usage
To run the code, use this command in the Raspberry:
```sh
python3 Session06.py
```

## Problem Proposed
This program **simulates a vehicle using sensors and actuators**. It is programmed in Python and requires a Raspberry Pi and a breadboard to work. The functionalitites implemented are the following:

- **On and off system:** Using a button, you can turn the vehicle on and off at any time.

- **Lighting system:** 2 LEDs respond to the state of the vehicle reacting as it corresponds when it is stopping, turning to the left or to the right (blinkers) or in a dark place.

- **Proximity checking:** The vehicle uses an ultrasonic sensor to check at all times if there is an object in front of it. It there is an object closer than 20 cm, it will stop the vehicle.

- **Command response:** The vehicle receives a JSON file with commands to execute. These will indicate the steering angle (performed using a servomotor), the speed (controlled using a DC motor) and the time the vehicle has to follow the instructions.

All of these work together to provide a simulation of how a vehicle would react to different orders.

## License
This project is licensed under the **MIT License**. This means you are free to use, modify, and distribute the software, but you must include the original license and copyright notice in any copies or substantial portions of the software.

## Contact
If necessary, contact the owner of this repository.
