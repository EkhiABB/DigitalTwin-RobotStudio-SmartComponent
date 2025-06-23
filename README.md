# DigitalTwin-RobotStudio-SmartComponent
A RobotStudio SmartComponent that monitors joint positions and I/O signals of a real ABB robot controller.

This SmartComponent is designed to use RobotStudio simulation as a connected digital twin, mirroring the real machine's state in real time.
This SmartComponent is intended to be used within a RobotStudio simulation while connected to a real controller.

Input Parameters:
IP address of the real robot controller
Mechanism in the station corresponding to the robot model
To monitor an I/O signal, type the name of the signal and click "Add IO Signal".

While the simulation is running, the SmartComponent will:
Read the joint values from the real robot and update the corresponding model in the station
Reflect the I/O signal values from the real controller in the simulation
