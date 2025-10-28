package frc.robot.subsystems.output;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.Constants;

public class OutputIOSim implements OutputIO {
  private SparkMax motor;
  private DCMotor neo;
  private SparkMaxSim sim;

  public OutputIOSim() {
    // Setup devices
    motor = new SparkMax(37, MotorType.kBrushless);
    neo = DCMotor.getNEO(1);
    sim = new SparkMaxSim(motor, neo);

    // Set brake mode
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    motor.configure(config, null, null);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(OutputIOInputs inputs) {
    inputs.velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(sim.getVelocity());
    sim.iterate(
        inputs.velocityRadPerSec, RoboRioSim.getVInVoltage(), Constants.LOOP_PERIOD_SECONDS);
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
