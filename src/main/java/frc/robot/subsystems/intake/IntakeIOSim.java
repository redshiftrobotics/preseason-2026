package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class IntakeIOSim implements IntakeIO {
  private DCMotor motor1;
  private DCMotor motor2;
  private DCMotorSim sim1;
  private DCMotorSim sim2;

  public IntakeIOSim() {
    motor1 = DCMotor.getNEO(1);
    motor2 = DCMotor.getNEO(1);

    sim1 = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor1, 0.004, 1.0), motor1);
    sim2 = new DCMotorSim(LinearSystemId.createDCMotorSystem(motor2, 0.004, 1.0), motor2);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.leftPositionRad = sim1.getAngularPositionRad();
    inputs.leftVelocityRadPerSec = sim1.getAngularVelocityRadPerSec();
    inputs.leftAppliedVolts = new double[]{0.0};
    inputs.leftSupplyCurrentAmps = new double[]{sim1.getCurrentDrawAmps()};

    inputs.rightPositionRad = sim2.getAngularPositionRad();
    inputs.rightVelocityRadPerSec = sim2.getAngularVelocityRadPerSec();
    inputs.rightAppliedVolts = new double[]{0.0};
    inputs.rightSupplyCurrentAmps = new double[]{sim2.getCurrentDrawAmps()};
  }

  @Override
  public void set(double speed) {
    sim1.setAngularVelocity(speed);
    sim2.setAngularVelocity(speed);
  }

  @Override
  public void stop() {
    sim1.setAngularVelocity(0);
    sim2.setAngularVelocity(0);
  }
}
