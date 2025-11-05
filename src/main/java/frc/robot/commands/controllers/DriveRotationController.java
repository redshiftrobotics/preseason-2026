package frc.robot.commands.controllers;

import static frc.robot.subsystems.drive.DriveConstants.HEADING_CONTROLLER_CONFIG;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

/** Controller for rotating robot to goal heading using ProfiledPIDController */
public class DriveRotationController {

  private final Drive drive;
  private Supplier<Rotation2d> goalSupplier;

  private final PIDController controller =
      new PIDController(
          HEADING_CONTROLLER_CONFIG.pid().kP(),
          HEADING_CONTROLLER_CONFIG.pid().kI(),
          HEADING_CONTROLLER_CONFIG.pid().kD());

  /**
   * Creates a new DriveRotationController.
   *
   * @param drive The drive subsystem to control.
   */
  public DriveRotationController(Drive drive) {
    this(drive, () -> null);
  }

  /**
   * Creates a new DriveRotationController.
   *
   * @param drive The drive subsystem to control.
   * @param setpointSupplier A supplier that provides the desired setpoint heading. Can supply null
   *     if there is no new goal, in which case the controller will hold the last goal.
   */
  public DriveRotationController(Drive drive, Supplier<Rotation2d> setpointSupplier) {
    this.drive = drive;
    this.goalSupplier = setpointSupplier;

    controller.enableContinuousInput(-Math.PI, Math.PI);
    controller.setTolerance(HEADING_CONTROLLER_CONFIG.toleranceRadians());

    reset();
  }

  public void setGoalSupplier(Supplier<Rotation2d> goalSupplier) {
    this.goalSupplier = goalSupplier;
  }

  public void setGoal(Rotation2d goal) {
    controller.setSetpoint(goal.getRadians());
  }

  /**
   * Resets the heading controller to the current robot heading and omega speed.
   *
   * <p>This is typically called at the start of a new command to ensure the controller starts from
   * the current heading.
   */
  public void reset() {
    controller.setSetpoint(drive.getRobotPose().getRotation().getRadians());
    controller.reset();
  }

  /**
   * Calculates the output of the heading controller.
   *
   * @return The angular velocity command in radians per second.
   */
  public double calculate() {
    Rotation2d setpoint = goalSupplier.get();

    if (setpoint != null) {
      controller.setSetpoint(setpoint.getRadians());
    }

    Rotation2d measured = drive.getRobotPose().getRotation();

    return controller.calculate(measured.getRadians());
  }

  /**
   * Returns whether the controller has reached the setpoint rotation.
   *
   * @return True if at setpoint, false otherwise.
   */
  public boolean atSetpoint() {
    return controller.atSetpoint();
  }
}
