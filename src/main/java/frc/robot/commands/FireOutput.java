package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.output.Output;

public class FireOutput extends Command {
  private final Output output;
  private final double velocityRadPerSec;
  private final double fireTimeSeconds;
  private Timer timer;

  public FireOutput(Output output, double velocityRadPerSec, double fireTimeSeconds) {
    this.output = output;
    this.velocityRadPerSec = velocityRadPerSec;
    this.fireTimeSeconds = fireTimeSeconds;
    timer = new Timer();
    addRequirements(output);
  }

  @Override
  public void initialize() {
    timer.restart();
    output.setVelocity(velocityRadPerSec);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(fireTimeSeconds);
  }

  @Override
  public void end(boolean interrupted) {
    output.stop();
    timer.stop();
  }
}
