# Robot Control Interface
Common robot control interface to be implemented by device specific plug-ins

## The Joint/Axis Rationale

Sometimes, when controlling a robot, the positions/velocities/interaction forces of interest aren't the ones over which we have direct control, the actuated joints, like when there is a 2-link robotic arm with motors on each revolution joint but we wish to control the cartesian (**x**,**y**) position of the end-effector tool. 

<p align="center">
  <img src="http://www.intechopen.com/source/html/25813/media/fig2.png" width="600"/>
</p>

In those cases, simple per actuator control isn't enough, as the variables of interest depend on the combination of all **joint** values. There's need for a higher-level control pass, that considers the robot kinematics/dynamics as a whole.

In **Robot Control Interface**, those controlled robot variables/coordinates are named **axes** (as an analogy with joystick/gamepad axes that control motion). The higher-level control pass is responsible for performing conversions **from joint to axis space for measurement**, and **from axis to joint space for actuator control setpoint calculation**. It also involves things like **joint impedance estimation and modulation based on axes data** and **reading of extra sensors, that aren't components of joint actuators (like contact ones)**.

With this, it is intended that robot control internals (for each implementation) are abstracted away, leaving the user/client to only worry about reading from joint/axes and writing to axes variables.

As, from user's point of view, **axis** or **joint** control is fundamentally the same, they alse use [the same structure](https://labdin.github.io/Robot-Control-Interface/structRobotVariables.html) of [double precision floating-point](https://en.wikipedia.org/wiki/Double-precision_floating-point_format) control variables:

  Position   |   Velocity   |    Force     | Acceleration |   Inertia    |  Stiffness   |   Damping
:----------: | :----------: | :----------: | :----------: | :----------: | :----------: | :----------:
   8 bytes   |    8 bytes   |   8 bytes    |   8 bytes    |   8 bytes    |   8 bytes    |   8 bytes 

## Usage

**Robot Control Interface** consists of a single header of common variables and functions declaration. Simply include it in both plug-in and host projects

## Documentation

Doxygen-generated detailed methods documentation available on a [GitHub Page](https://labdin.github.io/Robot-Control-Interface/robot__control_8h.html)

