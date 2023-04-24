# MechanicalDesignPortfolio

## 1. Design of Compact High-Speed flexure based Micro 3D Printer ![link](https://www.example.com)

Contribution: Individual

This printer has compliant X and Y stages with RLS position encoders and
contactless voice coil actuator. The laser is guided through mirrors to the
resin plate to create small parts ~20um accuracy. ANSYS used to find
displacement of compliant beams.

##  2. V-REX 80 ![link](https://www.example.com)
A trussed device to lift heavy objects for small scale industries, with forklift attachment.
Contribution: Team, kinematic design, actuator sizing, mechanical design,
off-the-shelf part selection and prototyping.
The device works with 2 linear actuators and a 12V battery and can lift 80kg
(180 lb.) with minimum human effort. This project involved significant ANSYS
FEA because of lifting pallet loads, to estimate deflection and stresses.

## 3. Design, Development and Fabrication of a Series Elastic Actuator ![link](https://www.example.com)
Contribution: Individual
Designed a module to fit on the servo motor to reduce jerk impact. It
incorporated non-contact rotary encoders (Bourns AMS) and a torsional
spring to estimate torque. Prototyping involved Laser cutting, Turning and
Milling.

## 4. RoboXO - A tic-tac-toe playing Robot ![link](https://www.example.com)
Contribution: Team, I contributed to ideation (kinematic), CAD parts
assembly, forward and inverse kinematics, control through MATLAB.
4 DoF robot with electromagnet end effector. X movement is on parallel
rails, Y and Z through the serial arm. Each joint has a Dynamixel servo motor,
with a relay controlling the magnet EE.
![link](https://www.example.com)
![link](https://www.example.com)

# Control Portfolio:
## 5. Double Inverted Pendulum control [link](https://www.example.com)
Grey box System Identification for motor, pendulum, and driver to update
motor model. Code generation and deployment to TI C2000 controller and
Maxon motor using Simulink embedded coder for data acquisition (DAQ).
Implemented discrete PID and State Observer Feedback control
(LQR/Kalman) for a comparative study.

## 6. Industrobot4.0: Control of Pick and Place PUMA 560 Robot [link](https://www.example.com)
Implemented Object detection and multi-axes 3D LSPB trajectory generation
using Robotics Toolbox. Develop custom function blocks for inertia, gravity,
and Coriolis forces in Simulink. Minimize norm error by PID control gain
tuning of feedforward, PD, and Inverse Dynamics control.
