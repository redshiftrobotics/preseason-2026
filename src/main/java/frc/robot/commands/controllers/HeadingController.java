package frc.robot.commands.controllers;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.HEADING_CONTROLLER_CONFIG;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;

/** Controller for rotating robot to goal heading using ProfiledPIDController */
public class HeadingController {

  private static final double SECONDS_TILL_IDLE = Constants.LOOP_PERIOD_SECONDS * 2;

  private final Drive drive;

  private final ProfiledPIDController controller =
      new ProfiledPIDController(
          HEADING_CONTROLLER_CONFIG.pid().kP(),
          HEADING_CONTROLLER_CONFIG.pid().kI(),
          HEADING_CONTROLLER_CONFIG.pid().kD(),
          new TrapezoidProfile.Constraints(
              DRIVE_CONFIG.maxAngularVelocity(), DRIVE_CONFIG.maxAngularAcceleration()),
          Constants.LOOP_PERIOD_SECONDS);

  private Supplier<Rotation2d> setpointSupplier;

  private Timer resetTimer = new Timer();

  public HeadingController(Drive drive, Supplier<Rotation2d> setpointSupplier) {
    this.drive = drive;
    this.setpointSupplier = setpointSupplier;

    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(HEADING_CONTROLLER_CONFIG.toleranceRadians());
  }

  /**
   * Resets the heading controller to the current robot heading and omega speed.
   *
   * <p>This is typically called at the start of a new command to ensure the controller starts from
   * the current heading.
   */
  public void reset() {
    controller.reset(
        drive.getRobotPose().getRotation().getRadians(),
        drive.getRobotSpeeds().omegaRadiansPerSecond);
  }

  public void setSetpointSupplier(Supplier<Rotation2d> setpoint) {
    this.setpointSupplier = setpoint;
  }

  public double calculate() {
    if (resetTimer.hasElapsed(SECONDS_TILL_IDLE)) {
      reset();
    }
    resetTimer.restart();

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
    double measurement = drive.getRobotPose().getRotation().getRadians();
    return MathUtil.isNear(
        measurement, controller.getGoal().position, controller.getPositionTolerance());
  }

  /**
   * Checks if the controller is at the goal heading in radians.
   *
   * <p>This method uses the controller's internal atGoal() method to determine if the controller
   * has reached the goal heading. Calculate must be called before this method to ensure the
   * controller has been updated with the latest measurements.
   *
   * @return true if the controller is at the goal heading, false otherwise.
   */
  public boolean controllerAtGoal() {
    return controller.atGoal();
  }

  public double getError() {
    return controller.getPositionError();
  }
}
