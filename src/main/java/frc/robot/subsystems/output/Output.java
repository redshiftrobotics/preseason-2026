package frc.robot.subsystems.output;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/** Subsystem for the output mechanism */
public class Output extends SubsystemBase {
  private final OutputIO io;
  private final OutputIOInputsAutoLogged inputs = new OutputIOInputsAutoLogged();

  public Output(OutputIO io) {
    this.io = io;
    io.configurePID(OutputConstants.PID_CONFIG.kP(), OutputConstants.PID_CONFIG.kI(), OutputConstants.PID_CONFIG.kD());
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  /**
   * Run the output in a closed-loop configuration at the specified velocity (radians per second)
   */
  public void setVelocity(double velocityRadPerSec) {
    io.setVelocity(velocityRadPerSec);
    Logger.recordOutput("Output/SetpointVelocity", velocityRadPerSec);
  }

  /** Stop the output and brake */
  public void stop() {
    setVelocity(0);
    io.stop();
  }

  /** Get the current velocity of the output in radians per second */
  @AutoLogOutput
  public double getVelocityRadPerSec() {
    return inputs.velocityRadPerSec;
  }
}
