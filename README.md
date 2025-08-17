
# FRC Swerve Project

FRC swerve drive implantation with Simulation and support for SparkMax and TalonFX swerve.

![Simulation of PathPlanner auto](https://github.com/user-attachments/assets/c266b861-9b09-45b3-a346-0fe8aa7c53b7)

## Drive Subsystem Structure 

<img width="500" align="right" alt="IO Layer Diagram" src="https://github.com/user-attachments/assets/4af16fb7-9e0d-4936-b4a5-197d76bf141f" />

Simulated and hardware implantations with same code using [AdvantageKit](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces) style IO layers.

<ol>
  <li>Drive subsystem does kinematics and odometry.</li>
  
  <li>Modules send motor requests and report module state.</li>
  
  <li>Module implantations for sim, SparkMax, and TalonFX motor controllers.</li>
</ol>

<br clear="right"/>

## Path Following

<img width="262"  align="right" alt="PathPlanner Path" src="https://github.com/user-attachments/assets/5cd3b558-cc02-4c64-b1a7-37ac9434c72f" />
<img height="400"  align="right" alt="First Path on 2024 bot" src="https://github.com/user-attachments/assets/51adb902-5abe-4c4c-bdb0-0999041a0d14" />
<p>
  Path following with PathPlannerLib.

  Video shows the first run of a path on our teams 2024 robot.
</p>

<br clear="right"/>

## Driving and Dashboard

https://github.com/user-attachments/assets/1c223f33-7faa-4e3f-b4a0-a05aa68be110

<img alt="Driving Control Scheme" src="https://github.com/user-attachments/assets/6b437824-4644-4ec0-b994-04d90ce4a73f" />
