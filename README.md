# MechanicalDesignPortfolio

## 1. [Design of Compact High-Speed flexure based Micro 3D Printer] (https://drive.google.com/file/d/1S0hZVuJbHFU6S8SvaRywW3aEAxqxxq-C/view?usp=sharing)

Contribution: Individual

This printer has compliant X and Y stages with RLS position encoders and
contactless voice coil actuator. The laser is guided through mirrors to the
resin plate to create small parts ~20um accuracy. ANSYS used to find
displacement of compliant beams.
![image](https://github.com/shubhamwani376/MechanicalDesignPortfolio/blob/main/Reference/iitb.png)

##  2. V-REX 80 
A trussed device to lift heavy objects for small scale industries, with forklift attachment.
Contribution: Team, kinematic design, actuator sizing, mechanical design,
off-the-shelf part selection and prototyping.
The device works with 2 linear actuators and a 12V battery and can lift 80kg
(180 lb.) with minimum human effort. This project involved significant ANSYS
FEA because of lifting pallet loads, to estimate deflection and stresses.

![image](https://github.com/shubhamwani376/MechanicalDesignPortfolio/blob/main/Reference/vrex1.png)
![image](https://github.com/shubhamwani376/MechanicalDesignPortfolio/blob/main/Reference/vrex2.png)

## 3. [Design, Development and Fabrication of a Series Elastic Actuator] (https://drive.google.com/file/d/1mJYNtMy62UZQn2ozjJne5kkfdMDKdUJf/view?usp=sharing)
Contribution: Individual
Designed a module to fit on the servo motor to reduce jerk impact. It
incorporated non-contact rotary encoders (Bourns AMS) and a torsional
spring to estimate torque. Prototyping involved Laser cutting, Turning and
Milling.
![image](https://github.com/shubhamwani376/MechanicalDesignPortfolio/blob/main/Reference/iisc.jpg)
![image](https://github.com/shubhamwani376/MechanicalDesignPortfolio/blob/main/Reference/iisc2.jpg)

## 4. [RoboXO - A tic-tac-toe playing Robot] (https://github.com/shubhamwani376/RoboXO)
Contribution: Team, I contributed to ideation (kinematic), CAD parts
assembly, forward and inverse kinematics, control through MATLAB.
4 DoF robot with electromagnet end effector. X movement is on parallel
rails, Y and Z through the serial arm. Each joint has a Dynamixel servo motor,
with a relay controlling the magnet EE.
![image](https://github.com/shubhamwani376/MechanicalDesignPortfolio/blob/main/Reference/roboxo.png)


# Control Portfolio:
## 5. [Double Inverted Pendulum control] (https://github.com/shubhamwani376/DigitalMotionControl)
Grey box System Identification for motor, pendulum, and driver to update
motor model. Code generation and deployment to TI C2000 controller and
Maxon motor using Simulink embedded coder for data acquisition (DAQ).
Implemented discrete PID and State Observer Feedback control
(LQR/Kalman) for a comparative study.

## 6. [Industrobot4.0: Control of Pick and Place PUMA 560 Robot](https://www.github.com/shubhamwani376/PUMA560_Industrial_Sorting_Robot)
Implemented Object detection and multi-axes 3D LSPB trajectory generation
using Robotics Toolbox. Develop custom function blocks for inertia, gravity,
and Coriolis forces in Simulink. Minimize norm error by PID control gain
tuning of feedforward, PD, and Inverse Dynamics control.
