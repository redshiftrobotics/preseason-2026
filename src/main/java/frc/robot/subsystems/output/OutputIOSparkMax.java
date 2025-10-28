package frc.robot.subsystems.output;

import com.revrobotics.spark.SparkBase.ControlType;
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
    motor.configure(config, null, null);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(OutputIOInputs inputs) {
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
  }

  /** Sets the velocity of the motor */
  @Override
  public void setVelocity(double velocityRadPerSec) {
    motor
        .getClosedLoopController()
        .setReference(
            Units.radiansPerSecondToRotationsPerMinute(velocityRadPerSec), ControlType.kVelocity);
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
    motor.configure(config, null, null);
  }
}
