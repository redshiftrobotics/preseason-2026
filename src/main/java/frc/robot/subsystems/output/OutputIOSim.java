package frc.robot.subsystems.output;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class OutputIOSim implements OutputIO {
  private DCMotor motor;
  private DCMotorSim sim;
  private PIDController pid;

  private double ffVolts = 0.0;

  public OutputIOSim() {
    // Setup devices
    motor = DCMotor.getNEO(1);
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.004, 1.0), motor);
    pid = new PIDController(0, 0, 0);
  }

  /** Updates the set of loggable inputs. */
  @Override
  public void updateInputs(OutputIOInputs inputs) {
    // Calculate input from PID controller
    double appliedVolts =
        MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
    sim.setInputVoltage(appliedVolts);

    // Update simulation
    sim.update(Constants.LOOP_PERIOD_SECONDS);
    inputs.velocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(sim.getAngularVelocityRadPerSec());
    inputs.appliedVolts = appliedVolts;
  }

  /** Sets the velocity of the motor */
  @Override
  public void setVelocity(double velocityRadPerSec, double ffVolts) {
    pid.setSetpoint(velocityRadPerSec);
    this.ffVolts = ffVolts;
  }

  /** Stop the motor and enable the brake */
  @Override
  public void stop() {
    pid.setSetpoint(0);
    sim.setInputVoltage(0);
  }

  /** Configure the PID constants */
  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
