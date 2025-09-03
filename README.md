
# FRC Swerve Project [![CI](https://github.com/MichaelLesirge/frc-swerve-drive/actions/workflows/main.yml/badge.svg)](https://github.com/MichaelLesirge/frc-swerve-drive/actions/workflows/main.yml)

This is a template FRC swerve drive implementation with simulation support and hardware support for both SparkMax and TalonFX motor controllers. It was developed by Team 8032 as a base for our robots, but is optionally open for use by any team.

![Simulation of PathPlanner auto](https://github.com/user-attachments/assets/c266b861-9b09-45b3-a346-0fe8aa7c53b7)

## Drive Subsystem Structure

<img width="500" align="right" alt="IO layer diagram" src="https://github.com/user-attachments/assets/4af16fb7-9e0d-4936-b4a5-197d76bf141f" />

Simulated and hardware implementations with same codebase using [AdvantageKit](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces) style IO layers and logging.

<ol>
  <li>Drive subsystem handles kinematics and odometry. Converts desired speeds to module states. Converts gyroscope data and module positions to chassis position.</li>

  <li>Modules optimize the module state and convert it to motor controller PID setpoints. Read IO data to determine module state.</li>

  <li>ModuleIOs for simulation and to interface with SparkMax or TalonFX motor controllers.</li>
</ol>

<br clear="right"/>

## Path Following

<img width="262" align="right" alt="PathPlanner path" src="https://github.com/user-attachments/assets/5cd3b558-cc02-4c64-b1a7-37ac9434c72f" />
<img height="400" align="right" alt="First path ran on 2024 bot" src="https://github.com/user-attachments/assets/51adb902-5abe-4c4c-bdb0-0999041a0d14" />
<p>
  Path following with PathPlannerLib.

  Video shows the first run of a path on our teams 2024 robot.

  This allows even new software team members to contribute effectively by creating and tuning paths without needing to write low-level robot code
</p>


<br clear="right"/>

## Driving and Dashboard

We use a driving and operating Xbox controller for our robot. This template only includes driving functionality so only one is needed here.

We drive field-relative, so when the translation stick is pushed away from you, the robot goes away from you no matter which way the "front" of the robot is facing. This makes it easier for the driver to take full advantage of the capabilities of swerve, including rotating and dodging on the move.

The robot knows it field-relative direction though a gyroscope placed in its center.

https://github.com/user-attachments/assets/1c223f33-7faa-4e3f-b4a0-a05aa68be110

<details>
  <summary>Driving Control Scheme</summary>
  <img alt="Driving Xbox controller labeled diagram" src="https://github.com/user-attachments/assets/8af4fa84-77a4-43d4-b75a-b5dbd6db9611" />
  <p>Diagram created with my Xbox <a href="https://michaellesirge.github.io/simple-web-projects/xbox-diagram-maker">controller labeling tool</a></p>

  <ul>
    <li>The left stick controls robot translation (forward/backward/left/right)</li>
    <li>The right stick controls robot rotation. By default pushing it left or right controls angular velocity.</li>
    <li>When RB is held the right stick controls the heading that you want the robot to face, for example pushing the stick up will cause the robot to rotate to field-relative 0 degrees.
    <li>By default the robot is field-relative, but Y can be pressed to toggle to robot-relative</li>
    <li>Menu button can be held for 3 seconds to reset odometry rotation & field-relative forward.</li>
    <li>D-pad is used to make small robot-relative adjustments.</li>
    <li>X stops the robots and turns the swerve modules to face inwards making an X shaped arrangement. This helps prevent the robot from moving when shoved.</li>
    <li>B cancels any movement related commands, for example canceling auto alignment to return manual control.</li>
  </ul>
</details>

## Vision

Vision processing is handled on OrangePi coprocessors running PhotonVision on the robot, but can also be simulated in a virtual 3D environment. We use April Tags to estimated the position of the robot for auto and alignment. We simulate vision to test camera placement options, filtering algorithms, and tag-relative auto alignment commands.

This video shows a single dummy camera in a simulated 2025 game field, with auto alignment code from that year. (Our 2025 robot had 4 cameras in reality).

Purple ghost robots represent accepted vision pose estimates, while yellow are rejected. Accepted vision estimates are combined with wheel odometry in Kalman filter.

https://github.com/user-attachments/assets/a532b2e0-d504-4c33-b588-d6f033edd90e


## Credits
* [AdvantageKit TalonFX Swerve Template](https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template) – source for TalonFX IO layer
* [Team 6328 Robot Code](https://github.com/Mechanical-Advantage/RobotCode2025Public) – drive subsystem, but mostly redone for leaning
* [Team 2930 Robot Code](https://github.com/FRC-Sonic-Squirrels/2025-Robot-Code/tree/main?tab=readme-ov-file) – vision subsystem inspiration
* [PhotonVision](https://photonvision.org) – coprocessor vision software
* [WPILib](https://github.wpilib.org/) – FRC robot library
* [SDS](https://www.swervedrivespecialties.com/) – module hardware


![Swerve chassis on side](https://github.com/user-attachments/assets/cc41ea92-382c-4d3a-8fee-30db69363e1f)
