package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class StopIntake extends Command {
  private final Intake intake;

  public StopIntake(Intake intake) {
    this.intake = intake;
  }

  @Override
  public void initialize() {
    intake.stop();
  }

  @Override
  public boolean isFinished() {
    return true;     
  }
}