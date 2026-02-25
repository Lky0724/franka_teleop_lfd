# franka_teleop_lfd

### Introduction

This project is developed based on libfranka and [franka_ros](https://github.com/frankarobotics/franka_ros), a ROS interface for Franka research robots. 

This project implements teleoperation with force feedback and learning from demonstration with GMM  on robot arm (follower, Franka Emika Panda) and haptic device (leader, Novint Falcon).

The repository contains two ROS workspaces /leader and /follower. Each of them should run on an individual PC. Leader PC takes command from haptic device, while follower PC receives command and control the robot arm. The two PCs are connected through LAN and share the roscore running on follower PC.

A complete system architecture is shown below:
![alt text](<system structure2.png>)

For instruction on installation and running the code, please refer to [Instruction](instruction.md).


### Documentation and resources

#### Presentation:

PDF: https://drive.google.com/file/d/1o8jt2Z-Th6HaCD0bHrKtlN_y2dwXz73T/view?usp=drive_link

PPT with sound recording: https://drive.google.com/file/d/1LfDFCoQf-sGwGpF493i-7mjOoRmpwXF3/view?usp=drive_link

#### Demonstration Videos:

Teleoperation (task board): https://drive.google.com/file/d/1X_Wn4lb8KlVR0nRx26T8BVr2dhPwmAwf/view?usp=drive_link

Teleoperation (orientation): https://drive.google.com/file/d/1Fs-HQTQBVaPDDeFXQqH15B6hOV1TXvPo/view?usp=drive_link

LfD (teaching): https://drive.google.com/file/d/1ZybiVj235hOm5NuY4P6yW_tAwtpwRaV4/view?usp=drive_link

LfD (reproducing): https://drive.google.com/file/d/1rHW_FaXfECiejPMdE_EVK8i1DROt7M4m/view?usp=drive_link



### Acknowledgement

This project is from course **Remote Machine Intelligence Lab** (25/26WS) at Technical University of Munich.

We would like to thank the supervisors and staff of the **Remote Machine Intelligence Lab** course at TUM for their guidance, hardware support, and valuable feedback throughout this project.

Kaiyuan Luo,
Feb 2026