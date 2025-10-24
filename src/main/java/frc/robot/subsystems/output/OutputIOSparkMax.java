package frc.robot.subsystems.output;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;

public class OutputIOSparkMax implements OutputIO {
  private SparkMax motor;

  public OutputIOSparkMax(int deviceID) {
    motor = new SparkMax(deviceID, MotorType.kBrushless);
  }

  /** Updates the set of loggable inputs. */
  public void updateInputs(OutputIOInputs inputs) {
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(motor.getEncoder().getVelocity());
  }

  /** Sets the velocity of the motor */
  public void setVelocity(double velocityRadPerSec) {}

  /** Stop the motor and enable the brake */
  public void stop() {}

  /** Configure the PID constants */
  public void configurePID(double kP, double kI, double kD) {}
}
