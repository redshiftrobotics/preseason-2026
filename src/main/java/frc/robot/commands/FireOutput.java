package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.output.Output;
import frc.robot.subsystems.output.OutputConstants;

public class FireOutput extends Command {
  private final Output output;
  private Timer timer;

  public FireOutput(Output output) {
    this.output = output;
    timer = new Timer();
    addRequirements(output);
  }

  @Override
  public void initialize() {
    timer.restart();
    output.setVelocity(OutputConstants.FIRE_VELOCITY_RAD_PER_SEC);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(OutputConstants.FIRE_TIME_SECONDS);
  }

  @Override
  public void end(boolean interrupted) {
    output.stop();
  }
  
}
