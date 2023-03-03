# CUSTOM ROS PID CONTROLLER for a FREELY MOVING ROBOT (6DOF)
#### The `main` branch focuses on position setpoints, the `irotate_edits` branch focuses on velocity setpoints.
#### The `irotate_edits` branch will be soon merged into the main branch. You can easily fork this repo and address your own changes. With the `irotate_edits` you can see also see how the project can be used for multiple robots at the same time.

## This repository is part of the [GRADE](https://eliabntt.github.io/GRADE-RR/home) project

The peculiarity of this controller is that it commands a robot in ROS Simulations by directly giving _JOINT VELOCITY COMMANDS_.

The controller is completely customizable. You can change the set of joints or add some. Check the example in the `irotate_edits` branch where we control `x-y-yaw-camera_yaw-wheel[0,1,2]`-joints.

The robot will be moved directly by the joints!

This controller is useful whenever you cannot control the robot as usual, like omnidirectional robots that need to move perpendicularly to the wheels or in not-fluidodynamics simulation [drones, submarines] or in any other case.

You can associate a PID controller to each JOINT or set the velocities directly.

You can define _for each joint_: limits (both position and velocity), PID+Anti windup configs.

You can either listen to a JointState topic to get the current state or you can subscribe to an odometry topic.

The main differences between the two branches are as follow:
- iRotate listen for two odometries and use sinchronized messages
- iRotate works with (local) velocity setpoints rather than global position ones
- iRotate shows how to control joints directly without the embedded PID controller (set speed instead of target).

### Main branch

joint_names: ["x_joint", "y_joint", "z_joint", "roll_joint", "pitch_joint", "yaw_joint"]

The publish rate is 100Hz.

### Installation

Simply download and build, you just need standard ros packages.

### Launch and config

You can launch the node with

`roslaunch custom_joint_controller_ros`

The main params are
```
<param name="odom" value="/$(arg namespace)/odom" />
<param name="setpoint" value="/$(arg namespace)/full_predicted_state" />
<param name="joint_command" value="/$(arg namespace)/joint_commands" />
```

`namespace` is the name of the robot. 

The config.yaml file contains the other settable parameters.

The `setpoint` is a full predicted state from a NMPC in my case. I then take the last point of the trajectory and set it as the goal of the PID. 
You can easily change that piece of code.

For reference I use the `nmpc` in `rotors_simulator` ([here](https://github.com/ethz-asl/rotors_simulator)) to compute the trajectory.

__________
### CITATION
If you find this work useful please cite our work as

```

```