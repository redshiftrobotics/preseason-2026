package frc.robot.commands.controllers;

import static frc.robot.subsystems.drive.DriveConstants.DRIVE_CONFIG;
import static frc.robot.subsystems.drive.DriveConstants.HEADING_CONTROLLER_CONFIG;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/** Controller for rotating robot to goal heading using ProfiledPIDController */
public class HeadingController {

  private final Drive drive;

  private final ProfiledPIDController headingControllerRadians;

  /**
   * Constructs a HeadingController with the specified drive subsystem and tolerance in degrees.
   *
   * @param drive The drive subsystem to control.
   */
  public HeadingController(Drive drive) {
    this.drive = drive;

    headingControllerRadians =
        new ProfiledPIDController(
            HEADING_CONTROLLER_CONFIG.pid().kP(),
            HEADING_CONTROLLER_CONFIG.pid().kI(),
            HEADING_CONTROLLER_CONFIG.pid().kD(),
            new TrapezoidProfile.Constraints(
                DRIVE_CONFIG.maxAngularVelocity(), DRIVE_CONFIG.maxAngularAcceleration()),
            Constants.LOOP_PERIOD_SECONDS);

    headingControllerRadians.enableContinuousInput(-Math.PI, Math.PI);

    headingControllerRadians.setTolerance(HEADING_CONTROLLER_CONFIG.toleranceRadians());
  }

  /**
   * Resets the heading controller to the current robot heading and omega speed.
   *
   * <p>This is typically called at the start of a new command to ensure the controller starts from
   * the current heading.
   */
  public void reset() {
    headingControllerRadians.reset(
        drive.getRobotPose().getRotation().getRadians(),
        drive.getRobotSpeeds().omegaRadiansPerSecond);
  }

  /**
   * Sets the goal heading for the controller.
   *
   * @param heading The desired goal heading as a Rotation2d.
   */
  public void setGoal(Rotation2d heading) {
    headingControllerRadians.setGoal(heading.getRadians());
  }

  /**
   * Sets the goal heading to the current robot heading. This is useful for commands that want to
   * maintain the current heading.
   */
  public void setGoalToCurrentHeading() {
    setGoal(drive.getRobotPose().getRotation());
  }

  /**
   * Resets the controller and sets the goal to the current robot heading.
   *
   * <p>This is typically called at the start of a new command to ensure the controller starts from
   * the current heading.
   */
  public void resetGoalToCurrentHeading() {
    reset();
    setGoalToCurrentHeading();
  }

  /**
   * Gets the current goal heading of the controller.
   *
   * @return The goal heading as a Rotation2d.
   */
  public Rotation2d getGoal() {
    return new Rotation2d(headingControllerRadians.getGoal().position);
  }

  /**
   * Calculates the output for the heading controller based on the current robot heading.
   *
   * @param goalHeadingRadians The desired goal heading in radians.
   * @return The calculated output for the controller.
   */
  public double calculate(Rotation2d goalHeadingRadians) {
    setGoal(goalHeadingRadians);
    return calculate();
  }

  /**
   * Calculates the output for the heading controller based on the current robot heading.
   *
   * @return The calculated output for the controller.
   */
  public double calculate() {
    // Calculate output
    double measurement = drive.getRobotPose().getRotation().getRadians();
    double output = headingControllerRadians.calculate(measurement);

    Logger.recordOutput(
        "Drive/HeadingController/Goal", headingControllerRadians.getGoal().position);
    Logger.recordOutput("Drive/HeadingController/Output", output);
    Logger.recordOutput(
        "Drive/HeadingController/HeadingError", headingControllerRadians.getPositionError());
    Logger.recordOutput("Drive/HeadingController/AtGoal", headingControllerRadians.atGoal());

    return output;
  }

  /** Checks if the robot heading is within the goal heading tolerance in radians. */
  public boolean atGoal() {
    double measurement = drive.getRobotPose().getRotation().getRadians();
    return MathUtil.isNear(
        measurement,
        headingControllerRadians.getGoal().position,
        headingControllerRadians.getPositionTolerance());
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
    return headingControllerRadians.atGoal();
  }

  public double getError() {
    return headingControllerRadians.getPositionError();
  }
}
