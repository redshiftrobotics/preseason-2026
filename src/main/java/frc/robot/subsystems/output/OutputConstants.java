package frc.robot.subsystems.output;

import frc.robot.Constants;

public final class OutputConstants {
  // Device ID for the hardware output motor
  // TODO: Get real device IDs to put here
  public static final int SPARKMAX_DEVICE_ID =
      switch (Constants.getRobot()) {
        case PHOENIX_TUNER_X -> 0;
        case CHASSIS_2025 -> 0;
        case CHASSIS_CANNON -> 0;
        case SIM_BOT -> 0;
      };

  public record PID(double kP, double kI, double kD) {}

  public record Feedforward(double kS, double kV, double kA) {}

  // PID constants for the closed-loop controller
  // TODO: Do tuning and get real constants to put here
  public static final PID PID_CONFIG =
      switch (Constants.getRobot()) {
        case PHOENIX_TUNER_X -> new PID(0, 0, 0);
        case CHASSIS_2025 -> new PID(0, 0, 0);
        case CHASSIS_CANNON -> new PID(0, 0, 0);
        case SIM_BOT -> new PID(0.6, 0, 0);
      };

  // Feedforward constants
  // TODO: Do tuning and get real constants to put here
  public static final Feedforward FF_CONFIG =
      switch (Constants.getRobot()) {
        case PHOENIX_TUNER_X -> new Feedforward(0, 0, 0);
        case CHASSIS_2025 -> new Feedforward(0, 0, 0);
        case CHASSIS_CANNON -> new Feedforward(0, 0, 0);
        case SIM_BOT -> new Feedforward(0.5, 5.25, 12);
      };

  // Constants for the launcher fire command
  // TODO: Get real values for these constants
  public static final double FIRE_VELOCITY_RAD_PER_SEC = 2 * Math.PI;
  public static final double FIRE_TIME_SECONDS = 5;
}
