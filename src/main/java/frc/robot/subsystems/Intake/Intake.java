package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.subsystems.intake.IntakeConstants.SPARKMAX_KD;
import static frc.robot.subsystems.intake.IntakeConstants.SPARKMAX_KI;
import static frc.robot.subsystems.intake.IntakeConstants.SPARKMAX_KP;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.intakeIO = io;
    intakeIO.configurePID(SPARKMAX_KP, SPARKMAX_KI, SPARKMAX_KD);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setVelocity(double velocityRadPerSec) {
    intakeIO.setVelocity(velocityRadPerSec);
  }

  public void stop() {
    intakeIO.stop();
  }
}