package frc.robot;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

/**
 * The Constants class provides a convenient place to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Time between loops in seconds, dt
  public static final double LOOP_PERIOD_SECONDS = Robot.defaultPeriodSecs; // 0.02

  public static final RobotType PRIMARY_ROBOT_TYPE = RobotType.PRESEASON_BOT;
  private static RobotType robotType;

  public static final boolean TUNING_MODE = true;

  public static final boolean HIDE_COMMAND_LOOP_OVERRUN = true;

  public static final boolean RUNNING_TEST_PLANS = false;

  public static RobotType getRobot() {
    if (robotType == null) {
      robotType = determineRobotType();
      if (robotType == null) {
        wrongRobotTypeFailedDetermination.set(true);
        robotType = PRIMARY_ROBOT_TYPE;
      }
    }
    if (RobotBase.isReal() && robotType == RobotType.SIM_BOT) {
      wrongRobotTypeAlertReal.set(true);
      robotType = PRIMARY_ROBOT_TYPE;
    }
    if (RobotBase.isSimulation() && robotType != RobotType.SIM_BOT) {
      wrongRobotTypeAlertSim.set(true);
      robotType = RobotType.SIM_BOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (getRobot()) {
      case SIM_BOT -> Mode.SIM;
      default -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    PRESEASON_BOT,
    CHASSIS_2025,
    CHASSIS_CANNON,
    SIM_BOT,
  }

  private static RobotType determineRobotType() {
    if (RobotBase.isReal()) {
      switch (RobotController.getSerialNumber()) {
        case "032D2143":
          return RobotType.CHASSIS_CANNON;
        case "02384981":
          return RobotType.CHASSIS_2025;
      }
    } else if (RobotBase.isSimulation()) {
      return RobotType.SIM_BOT;
    }
    return null;
  }

  private static final Alert wrongRobotTypeAlertReal =
      new Alert(
          String.format(
              "Invalid robot selected, using %s robot as default.", PRIMARY_ROBOT_TYPE.toString()),
          Alert.AlertType.kInfo);

  private static final Alert wrongRobotTypeAlertSim =
      new Alert(
          String.format(
              "Invalid robot selected for simulation robot, using simulation robot as default."),
          AlertType.kInfo);

  private static final Alert wrongRobotTypeFailedDetermination =
      new Alert(
          String.format(
              "Failed to determine robot from RoboRio serial number, using %s robot as default.",
              PRIMARY_ROBOT_TYPE.toString()),
          AlertType.kInfo);
}
