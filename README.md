# CUSTOM ROS PID CONTROLLER for a FREE FLOATING ROBOT

The peculiarity of this controller is that it commands a robot in ROS Simulations by directly giving JOINT VELOCITY COMMANDS.

The robot with a x-y-z-roll-pitch-yaw set of joints will receive commands to those DIRECTLY!

This controller is useful whenever you cannot control the robot with the usual way.

Each JOINT has a PID associated to it. 

You can define FOR EACH JOINT: limits (both position and velocity), PID+Anti windup configs.

You can either listen to a JointState topic to get the current state or you can subscribe to an odometry topic.

joint_names: ["x_joint", "y_joint", "z_joint", "roll_joint", "pitch_joint", "yaw_joint"]

The publish rate is 100Hz.

You can launch the node with

`roslaunch custom_joint_controller_ros`

The main params are
```
<param name="odom" value="/$(arg namespace)/odom" />
<param name="setpoint" value="/$(arg namespace)/full_predicted_state" />
<param name="joint_command" value="/$(arg namespace)/joint_commands" />
```

The config.yaml file contains the other settable parameters.

The `setpoint` is a full predicted state from a NMPC in my case. I then take the last point of the trajectory and set it as the goal of the PID. 
You can easily change that piece of code.

For reference I use the `nmpc` in `rotors_simulator` ([here](https://github.com/ethz-asl/rotors_simulator)) to compute the trajectory.
