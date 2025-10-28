package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetIntakeSpeed extends Command {
  private final Intake intake;
  private final double speed;

  public SetIntakeSpeed(Intake intake, double speed) {
    this.intake = intake;
    this.speed = speed;
  }

  @Override
  public void initialize() {
    intake.set(speed);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}