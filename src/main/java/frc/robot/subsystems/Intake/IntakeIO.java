package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public double leftPositionRad = 0.0;
    public double rightPositionRad = 0.0;
    public double leftVelocityRadPerSec = 0.0;
    public double rightVelocityRadPerSec = 0.0;

    public double[] leftAppliedVolts = new double[] {};
    public double[] rightAppliedVolts = new double[] {};
    public double[] leftSupplyCurrentAmps = new double[] {};
    public double[] rightSupplyCurrentAmps = new double[] {};
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void set(double speed) {}

  public default void stop() {}
}