# Design-and-Implementation-of-Digital-Twin-for-a-Line-Follower-Robot
Open-source digital twin for a line-follower robot, integrating MATLAB controller, Unity 3D virtual model, and ESP32 physical robot. Features PID control, UDP/TCP sync, and HTTP dashboard. For university students to modify hardware and external users to simulate or build their own robot. MIT License.

STEP 01: The setup for the Matlab simulation can be accessed through the Matlab Simulation application, which includes two models: one unified model that works for both virtual and physical simulations, and another individual model that operates the virtual and physical simulations separately, while utilizing the same control logic. Both models are self-designed and developed and feature a fully integrated communication system that uses UDP and TCP over Wi-Fi.

 STEP 02: The second step is to set up the Unity 3D simulation. You can do this by downloading the Unity file, which includes all the necessary scripts, models, and other components. Once you have everything, you can set it up and run the simulation to test it with a virtual individual controller. If it operates correctly, then that's great!

STEP 03: The third aspect is Physical. As a Riga Technical University student, you will have access to the physical model that I developed for further study and upgrades. If you are a student from another university, you will need to create the physical model using the ESP32 microcontroller script and the robot building manual. 

Alternatively, if you are interested in building something different and aim to create a digital twin, you can simply modify the design in the Unity application and adjust the ESP32 microcontroller script by selecting your desired board. This entire integrated synchronization system for the digital twin model is designed not only for the developed physical model but can also be adapted for any model by simply setting it up.
