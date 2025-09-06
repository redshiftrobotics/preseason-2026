package frc.robot.commands.controllers;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.ROTATION_CONTROLLER_CONSTANTS;
import static frc.robot.subsystems.drive.DriveConstants.ROTATION_TOLERANCE;
import static frc.robot.subsystems.drive.DriveConstants.TRANSLATION_CONTROLLER_CONSTANTS;
import static frc.robot.subsystems.drive.DriveConstants.TRANSLATION_TOLERANCE;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class TranslationController {

  private static final double SECONDS_TO_IDLE = 0.2;

  private final Drive drive;

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

  private Timer resetTimer = new Timer();

  public TranslationController(Drive drive, Supplier<Pose2d> setpointSupplier) {
    this.drive = drive;
    this.setpointSupplier = setpointSupplier;

    controller.setTolerance(
        new Pose2d(TRANSLATION_TOLERANCE, TRANSLATION_TOLERANCE, ROTATION_TOLERANCE));
  }

  public void reset() {
    resetTimer.restart();
    controller.getXController().reset();
    controller.getYController().reset();
    controller
        .getThetaController()
        .reset(
            drive.getRobotPose().getRotation().getRadians(),
            drive.getRobotSpeeds().omegaRadiansPerSecond);
  }

  public void setSetpointSupplier(Supplier<Pose2d> setpoint) {
    this.setpointSupplier = setpoint;
  }

  public ChassisSpeeds calculate() {
    if (resetTimer.hasElapsed(SECONDS_TO_IDLE)) {
      reset();
    } else {
      resetTimer.restart();
    }

    Pose2d setpoint = this.setpointSupplier.get();
    Pose2d measured = drive.getRobotPose();
    if (setpoint == null) {
      return new ChassisSpeeds(0, 0, 0);
    }
    return controller.calculate(measured, setpoint, 0, setpoint.getRotation());
  }

  @AutoLogOutput(key = "Drive/TranslationController/atGoal")
  public boolean atGoal() {
    Pose2d setpoint = this.setpointSupplier.get();
    Pose2d measured = drive.getRobotPose();
    if (setpoint == null) {
      return false;
    }
    return measured.getTranslation().getDistance(setpoint.getTranslation()) < TRANSLATION_TOLERANCE
        && MathUtil.isNear(
            measured.getRotation().getRadians(),
            setpoint.getRotation().getDegrees(),
            ROTATION_TOLERANCE.getRadians(),
            0,
            Math.PI * 2);
  }
}
