package frc.robot.subsystems.dashboard;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.AllianceMirrorUtil;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class DriverDashboard {

  private static final Field2d field = new Field2d();

  public static Supplier<String> currentDriveModeName = () -> "Unknown";

  public static Supplier<Pose2d> poseSupplier = () -> Pose2d.kZero;
  public static Supplier<SwerveModuleState[]> wheelStatesSupplier =
      () ->
          new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
          };
  public static Supplier<ChassisSpeeds> speedsSupplier = () -> new ChassisSpeeds();

  public static BooleanSupplier hasVisionEstimate = () -> false;
  private static final Debouncer hasVisionEstimateDebounce =
      new Debouncer(0.1, DebounceType.kFalling);

  public static void addSubsystem(SubsystemBase subsystem) {
    SmartDashboard.putData(subsystem);
  }

  public static void addCommand(String name, Runnable runnable, boolean runsWhenDisabled) {
    addCommand(name, Commands.runOnce(runnable), runsWhenDisabled);
  }

  public static void addCommand(String name, Command command, boolean runsWhenDisabled) {
    SmartDashboard.putData(name, command.withName(name).ignoringDisable(runsWhenDisabled));
  }

  public static Field2d getField() {
    return field;
  }

  public static void initDashboard() {
    SmartDashboard.putData("Field", field);
    SmartDashboard.putData(CommandScheduler.getInstance());

    SmartDashboard.putString("RobotName", Constants.getRobot().toString());
    SmartDashboard.putString("RobotRoboRioSerialNumber", RobotController.getSerialNumber());

    customWidgets();
  }

  public static void updateDashboard() {
    SmartDashboard.putNumber("Game Time", DriverStation.getMatchTime());

    Pose2d pose = poseSupplier.get();
    SmartDashboard.putNumber(
        "Heading Degrees", -AllianceMirrorUtil.apply(pose.getRotation()).getDegrees());
    field.setRobotPose(pose);

    ChassisSpeeds speeds = speedsSupplier.get();

    SmartDashboard.putNumber(
        "Speed MPH", Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) * 2.2369);

    SmartDashboard.putBoolean(
        "Has Vision", hasVisionEstimateDebounce.calculate(hasVisionEstimate.getAsBoolean()));

    SmartDashboard.putString("Drive Mode", currentDriveModeName.get());
  }

  private static void customWidgets() {
    String[] moduleNames = {"Front Left", "Front Right", "Back Left", "Back Right"};
    SmartDashboard.putData(
        "Swerve Drive",
        new Sendable() {
          @Override
          public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("SwerveDrive");

            for (int i = 0; i < 4; i++) {
              final int index = i;
              builder.addDoubleProperty(
                  moduleNames[i] + " Angle",
                  () ->
                      AllianceMirrorUtil.apply(wheelStatesSupplier.get()[index].angle).getRadians(),
                  null);
              builder.addDoubleProperty(
                  moduleNames[i] + " Velocity",
                  () -> wheelStatesSupplier.get()[index].speedMetersPerSecond,
                  null);
            }

            builder.addDoubleProperty(
                "Robot Angle",
                () -> AllianceMirrorUtil.apply(poseSupplier.get().getRotation()).getRadians(),
                null);
          }
        });
  }
}
