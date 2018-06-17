# EL6483_Group1
## EL-GY 6483 "Real Time Embedded Systems" - Final Project

### Group Members
|NAME            |NYU ID          |
|:---------------|:---------------|
|Tenzing Rabgyal | tr1440@nyu.edu |
|Mohammad Abbasi | mra413@nyu.edu |
|Deliang Wang    | dw2331@nyu.edu |
|John Lee        | jjl624@nyu.edu |

### Project Requirements

#### Objective
This project involves navigating a vehicle through a minefield by using audio beacons of fixed frequencies located throughout the field. The vehicle will start at a fixed location in the field. The mission is to locate the “next” audio beacon and steer the vehicle toward that beacon while monitoring for the next beacon. The vehicle should continue to search each consecutive beacon until the final beacon is located, indicating the vehicle has exited the minefield. During the journey, the vehicle may not collide with any of the beacons and must steer clear.
#### Rules
1. You may use preassembled libraries for PWM signal generation and audio processing.
2. One group will be tested at a time. The vehicle will be placed at the starting point, and the beacons will be turned on. There should be a “Go” button on the vehicle to start the journey to the finish line as well as a visual indication (LED) that the journey has ended.
3. The time calculation will begin when the team pressed the “Go” button and will conclude when the visual indication of completion is visible. The time recorded for each team is marked after any part of the vehicle passes the first edge of the finish line.
4. Each team will have 2 tries to complete the task.
5. All source code must be submitted.
#### Equipment List
* Teensy 3.2 Microcontroller (w/ 32-bit ARM processor)
* MAX4466 Microphone Amplifier ([Link](https://www.adafruit.com/product/1063))
* PS1240 Piezo Buzzer ([Link](https://www.adafruit.com/product/160))
* Robot Vehicle Chassis ([Link](https://www.amazon.com/gp/product/B06XZC2XDV/ref=oh_aui_detailpage_o02_s00?ie=UTF8&psc=1))
* Other Miscellaneous Items (Breadboards, Wires, Batteries, etc)
