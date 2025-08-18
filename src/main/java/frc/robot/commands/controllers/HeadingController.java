package frc.robot.commands.controllers;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.HEADING_CONTROLLER_CONFIG;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

public class HeadingController {

  private final Drive drive;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          HEADING_CONTROLLER_CONFIG.pid().kP(),
          HEADING_CONTROLLER_CONFIG.pid().kI(),
          HEADING_CONTROLLER_CONFIG.pid().kD(),
          new TrapezoidProfile.Constraints(
              DRIVE_CONFIG.maxAngularVelocity(), DRIVE_CONFIG.maxAngularAcceleration()),
          Constants.LOOP_PERIOD_SECONDS);

  private Supplier<Rotation2d> setpointSupplier = () -> null;

  public HeadingController(Drive drive) {
    this.drive = drive;

    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(HEADING_CONTROLLER_CONFIG.toleranceRadians());
  }

  public void reset() {
    controller.reset(
        drive.getRobotPose().getRotation().getRadians(),
        drive.getRobotSpeeds().omegaRadiansPerSecond);
  }

  public void setSetpointSupplier(Supplier<Rotation2d> setpoint) {
    this.setpointSupplier = setpoint;
  }

  public double calculate() {
    Rotation2d setpoint = setpointSupplier.get();
    Rotation2d measured = drive.getRobotPose().getRotation();
    if (setpoint == null) {
      return 0.0;
    }
    return controller.calculate(
        measured.getRadians(), new TrapezoidProfile.State(setpoint.getRadians(), 0.0));
  }

  @AutoLogOutput(key = "Drive/HeadingController/atGoal")
  public boolean atGoal() {
    Rotation2d setpoint = setpointSupplier.get();
    Rotation2d measured = drive.getRobotPose().getRotation();
    if (setpoint == null) {
      return false;
    }
    return MathUtil.isNear(
        measured.getRadians(),
        setpoint.getRadians(),
        HEADING_CONTROLLER_CONFIG.toleranceRadians(),
        0,
        Math.PI * 2);
  }
}
