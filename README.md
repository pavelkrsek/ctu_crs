# CRS robots control at CTU/CIIRC
Package to control CTU/CIIRC robots CRS93 and CRS97 via MARS control unit.

## Installation

```
pip install ctu_crs
```

## Simple python script to control the robot

```python
from ctu_crs import CRS93 # or CRS97
robot = CRS93()  # set argument tty_dev=None if you are not connected to robot,
# it will allow you to compute FK and IK offline
robot.initialize()  # initialize connection to the robot, perform hard and soft home
q = robot.get_q()  # get current joint configuration
robot.move_to_q(q + [0.1, 0.0, 0.0, 0.0, 0.0, 0.0])  # move robot all values in radians
robot.wait_for_motion_stop() # wait until the robot stops
robot.close()  # close the connection
```

## Step-by-Step Procedure for Operating the Robot

- **Power On the Robot**
Turn on the robot using the red switch on the front panel of the control unit (the switch will light up).
Press the yellow Arm Power button (a yellow LED will light up above it).
- **Initialize Communication and Setup in Python**
Run the following commands to initiate communication and perform necessary setup:
```python
from ctu_crs import CRS93 # or CRS97
robot = CRS93()  # set argument tty_dev=None if you are not connected to robot,
# it will allow you to compute FK and IK offline
robot.initialize()  # initialize connection to the robot, perform hard and soft home
```
- **Move the Robot** After initialization, you can continue with your desired commands for operating the robot.

- **End of Session - Return Robot to Home Position**
To finish working and return the robot to the home position, execute:
```python
robot.soft_home()
robot.close()  # close the connection
```
Turn off the robot by switching off the red rocker switch on the front panel.

### Accessing the Robot Within the Safety Cage
If you need to work inside the protective cage:
Stop the robot (wait until movement stops completely). You can check with:
```python
robot.in_motion()
```
Run the following command to release the robot:
```python
robot.release()
```
This will engage the brakes and disconnect feedback. Be cautious, as the robot arm might slightly drop, so avoid performing this command directly above the work surface.
Open the cage door (the yellow Arm Power LED will turn off).

## Emergency Stop Procedure
If the robot behaves unexpectedly, immediately press the Emergency Stop button (red mushroom-shaped button).

### Restoring Operation After Emergency Stop or Working Inside the Cage
Unlock the emergency stop by pressing the blue button on the side of the emergency button, or close the cage door if open.
Press the Motion Stop button.
Press the yellow Arm Power button (the yellow Arm Power LED will turn on).
Resume sending movement commands to the robot.
If the motor enters an error state (indicated by a flashing green status LED above the motor’s letter and a red error LED), reset the motor state with:
```python
robot.reset_motors()
```
Note: Re-homing the robot is not necessary, even after activating the emergency stop or opening the cage door, as long as the control unit is not powered off.