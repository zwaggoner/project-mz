# PID Controller/Input Shaper Project

Project to demonstrate PID control and use of an input shaper on a simple DC motor with a flexible rod attachment.

Assignment for school - goal was to use linux kernel modules. Hardware interface was written in using a linux kernel module for the motor, where the interface was ioctl. 

Actually controller logic was written in a user-space application made to run at high priority.
