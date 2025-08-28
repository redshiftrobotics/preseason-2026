package frc.robot.subsystems.dashboard;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DriverDashboard extends SubsystemBase {

  // --- Singleton Setup ---

  private static DriverDashboard instance;

  private final Field2d field = new Field2d();

  private DriverDashboard() {
    SmartDashboard.putData("Field", field);
    SmartDashboard.putData(CommandScheduler.getInstance());

    SmartDashboard.putString("RobotName", Constants.getRobot().toString());
    SmartDashboard.putString("RobotRoboRioSerialNumber", RobotController.getSerialNumber());
  }

  public static DriverDashboard getInstance() {
    if (instance == null) instance = new DriverDashboard();
    return instance;
  }

  // --- Fields ---

  private Supplier<Pose2d> poseSupplier;
  private Supplier<ChassisSpeeds> speedsSupplier;

  private BooleanSupplier hasVisionEstimate;
  private Debouncer debouncer;

  // --- Setters ---

  public void addSubsystem(SubsystemBase subsystem) {
    SmartDashboard.putData(subsystem);
  }

  public void addCommand(String name, Runnable runnable, boolean runsWhenDisabled) {
    addCommand(name, Commands.runOnce(runnable), runsWhenDisabled);
  }

  public void addCommand(String name, Command command, boolean runsWhenDisabled) {
    SmartDashboard.putData(name, command.withName(name).ignoringDisable(runsWhenDisabled));
  }

  public void setPoseSupplier(Supplier<Pose2d> robotPoseSupplier) {
    this.poseSupplier = robotPoseSupplier;
  }

  public void setRobotSupplier(Supplier<ChassisSpeeds> robotSpeedsSupplier) {
    this.speedsSupplier = robotSpeedsSupplier;
  }

  public void setHasVisionEstimateSupplier(BooleanSupplier hasVisionEstimate, double debounceTime) {
    this.hasVisionEstimate = hasVisionEstimate;
    debouncer = new Debouncer(debounceTime, DebounceType.kFalling);
  }

  public Field2d getField() {
    return field;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Game Time", DriverStation.getMatchTime());

    if (poseSupplier != null) {
      Pose2d pose = poseSupplier.get();
      SmartDashboard.putNumber("Heading Degrees", ((-pose.getRotation().getDegrees() + 360) % 360));
      field.setRobotPose(pose);
    }

    if (speedsSupplier != null) {
      ChassisSpeeds speeds = speedsSupplier.get();

      SmartDashboard.putNumber(
          "Speed MPH", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) * 2.2369);
    }
    
    if (hasVisionEstimate != null) {
      SmartDashboard.putBoolean(
          "Has Vision", debouncer.calculate(hasVisionEstimate.getAsBoolean()));
    }
  }
}
