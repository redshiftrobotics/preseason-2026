package frc.robot.subsystems.output;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class OutputIOSim implements OutputIO {
  private DCMotor motor;
  private DCMotorSim sim;
  private PIDController pid;

  private LoggedMechanism2d mechanism;
  private LoggedMechanismRoot2d mechRoot;
  private LoggedMechanismLigament2d ligament;

  private double ffVolts = 0.0;

  public OutputIOSim() {
    // Setup devices
    motor = DCMotor.getNEO(1);
    sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor, 0.004, 1.0), motor);
    pid = new PIDController(0, 0, 0);

    // Setup mechanism
    mechanism = new LoggedMechanism2d(200, 200);
    mechRoot = mechanism.getRoot("Output", 100, 100);
    ligament = mechRoot.append(new LoggedMechanismLigament2d("Spinner", 50, 0));
    ligament.setColor(new Color8Bit("#006600"));
    ligament.setLineWeight(12);
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
    inputs.positionRad = sim.getAngularPositionRad();
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();

    ligament.setAngle(Units.radiansToDegrees(inputs.positionRad));
    Logger.recordOutput("Output/Visualization", mechanism);
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
