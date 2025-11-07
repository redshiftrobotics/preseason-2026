package frc.robot.subsystems.output;

import org.littletonrobotics.junction.AutoLog;

/** General IO interface between the Output subsystem and the hardware */
public interface OutputIO {
  @AutoLog
  public static class OutputIOInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  default void updateInputs(OutputIOInputs inputs) {}

  /** Sets the velocity of the motor */
  default void setVelocity(double velocityRadPerSec, double ffVolts) {}

  /** Stop the motor and enable the brake */
  default void stop() {}

  /** Configure the PID constants */
  default void configurePID(double kP, double kI, double kD) {}
}
