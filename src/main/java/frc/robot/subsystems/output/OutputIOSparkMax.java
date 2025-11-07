package frc.robot.subsystems.output;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class OutputIOSparkMax implements OutputIO {
  private SparkMax motor;

  public OutputIOSparkMax(int deviceID) {
    motor = new SparkMax(deviceID, MotorType.kBrushless);
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(OutputIOInputs inputs) {
    inputs.positionRad = Units.rotationsToRadians(motor.getEncoder().getPosition());
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.supplyCurrentAmps = motor.getOutputCurrent();
  }

  /** Sets the velocity of the motor */
  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    motor
        .getClosedLoopController()
        .setReference(
            Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec),
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts,
            ArbFFUnits.kVoltage);
  }

  /** Stop the motor and enable the brake */
  @Override
  public void stop() {
    motor.stopMotor();
  }

  /** Configure the PID constants */
  @Override
  public void configurePID(double kP, double kI, double kD) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.closedLoop.pid(kP, kI, kD);
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
