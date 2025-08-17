
# FRC Swerve Project [![CI](https://github.com/MichaelLesirge/frc-swerve-drive/actions/workflows/main.yml/badge.svg)](https://github.com/MichaelLesirge/frc-swerve-drive/actions/workflows/main.yml)

First Robotics Competion swerve drive implementation with Simulation and support for SparkMax and TalonFX swerve. Meant as template project for my team (8032), but open to anyone.

![Simulation of PathPlanner auto](https://github.com/user-attachments/assets/c266b861-9b09-45b3-a346-0fe8aa7c53b7)

## Drive Subsystem Structure

<img width="500" align="right" alt="IO Layer Diagram" src="https://github.com/user-attachments/assets/4af16fb7-9e0d-4936-b4a5-197d76bf141f" />

Simulated and hardware implementations with same codebase using [AdvantageKit](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces) style IO layers and logging.

<ol>
  <li>Drive subsystem does kinematics and odometry. Robot speed and position interface.</li>

  <li>Modules send motor requests and report module state. Module state interface.</li>

  <li>Module implementations for sim, SparkMax, and TalonFX motor controllers.</li>
</ol>

<br clear="right"/>

## Path Following

<img width="262" align="right" alt="PathPlanner Path" src="https://github.com/user-attachments/assets/5cd3b558-cc02-4c64-b1a7-37ac9434c72f" />
<img height="400" align="right" alt="First Path on 2024 bot" src="https://github.com/user-attachments/assets/51adb902-5abe-4c4c-bdb0-0999041a0d14" />
<p>
  Path following with PathPlannerLib.

  Video shows the first run of a path on our teams 2024 robot.

  I worked hard to create path following code for our robot because it lets my inexperienced software team members still productively contribute to the robot.
</p>


<br clear="right"/>

## Driving and Dashboard

We use a driving and operating Xbox controller for our robot. This template only including driving function so only one is needed here.

We drive field-relative, so when the translation stick is pushed away from you, the robot goes away from you no matter which way the "front" of the robot is facing.

The robot knows it field relative direction though a gyroscope placed in its center.

https://github.com/user-attachments/assets/1c223f33-7faa-4e3f-b4a0-a05aa68be110

<img alt="Driving Control Scheme" src="https://github.com/user-attachments/assets/6b437824-4644-4ec0-b994-04d90ce4a73f" />

## Vision

Vision processing is done with OrangePi's running PhotonVision on the robot, but here it is simulated. We use AprilTags to estimated the position of the robot for auto and alignment. We simulate vision to test camera placement and filtering. 

Purple ghost robots are accepted vision pose estimates, yellow are rejected. Accepted vision estimates are combined with wheel odometry in Kalman filter.

This video shows a single dummy camera in a simulated 2025 game field, with auto alignment code from that year. (Our 2025 robot had 4 cameras in reality).

https://github.com/user-attachments/assets/a532b2e0-d504-4c33-b588-d6f033edd90e


## Credit
* TalonFX swerve module IO layer and odometry thread is from AdvantageKit's Talon Swerve Template [talonfx-swerve-template](docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template)
* The drive subsystem is inspired by FRC Team 6328, although almost fully reimplanted for learning. [Mechanical-Advantage/RobotCode2025Public](https://github.com/Mechanical-Advantage/RobotCode2025Public)
* The vision subsystem is inspired by FRC team 2930. [FRC-Sonic-Squirrels/2025-Robot-Code](https://github.com/FRC-Sonic-Squirrels/2025-Robot-Code/tree/main?tab=readme-ov-file)
* Photonvision for gathering information from the coprocessors [Photonvision](https://photonvision.org)
* [WPILib!](https://github.wpilib.org/)



