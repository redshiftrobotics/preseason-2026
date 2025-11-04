package frc.robot.commands.controllers;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.ROTATION_CONTROLLER_CONSTANTS;
import static frc.robot.subsystems.drive.DriveConstants.ROTATION_TOLERANCE;
import static frc.robot.subsystems.drive.DriveConstants.TRANSLATION_CONTROLLER_CONSTANTS;
import static frc.robot.subsystems.drive.DriveConstants.TRANSLATION_TOLERANCE;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** Controller for driving robot to goal pose using HolonomicDriveController */
public class DrivePoseController {

  private final Drive drive;

  private boolean goalReceived = false;

  private final HolonomicDriveController controller =
      new HolonomicDriveController(
          new PIDController(
              TRANSLATION_CONTROLLER_CONSTANTS.kP(),
              TRANSLATION_CONTROLLER_CONSTANTS.kI(),
              TRANSLATION_CONTROLLER_CONSTANTS.kD()),
          new PIDController(
              TRANSLATION_CONTROLLER_CONSTANTS.kP(),
              TRANSLATION_CONTROLLER_CONSTANTS.kI(),
              TRANSLATION_CONTROLLER_CONSTANTS.kD()),
          new ProfiledPIDController(
              ROTATION_CONTROLLER_CONSTANTS.kP(),
              ROTATION_CONTROLLER_CONSTANTS.kI(),
              ROTATION_CONTROLLER_CONSTANTS.kD(),
              DRIVE_CONFIG.getAngularConstraints()));

  private Supplier<Pose2d> setpointSupplier;

  public DrivePoseController(Drive drive, Supplier<Pose2d> setpointSupplier) {
    this.drive = drive;
    this.setpointSupplier = setpointSupplier;

    controller.setTolerance(
        new Pose2d(TRANSLATION_TOLERANCE, TRANSLATION_TOLERANCE, ROTATION_TOLERANCE));

    reset();
  }

  public void reset() {
    goalReceived = false;
    resetControllers();
  }

  private void resetControllers() {
    controller.getXController().reset();
    controller.getYController().reset();
    controller
        .getThetaController()
        .reset(
            drive.getRobotPose().getRotation().getRadians(),
            drive.getRobotSpeeds().omegaRadiansPerSecond);
  }

  public ChassisSpeeds calculate() {
    Pose2d setpoint = setpointSupplier.get();
    Pose2d measured = drive.getRobotPose();

    if (setpoint != null) {
      if (!goalReceived) {
        resetControllers();
      }
      goalReceived = true;
    }

    if (!goalReceived) {
      return new ChassisSpeeds();
    }

    return controller.calculate(measured, setpoint, 0, setpoint.getRotation());
  }

  @AutoLogOutput(key = "Drive/TranslationController/atGoal")
  public boolean atGoal() {
    return controller.atReference() && goalReceived;
  }
}
