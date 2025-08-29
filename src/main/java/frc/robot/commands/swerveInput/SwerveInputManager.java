package frc.robot.commands.swerveInput;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Deque;
import java.util.LinkedList;
import java.util.function.UnaryOperator;

public class SwerveInputManager {

  private final Deque<UnaryOperator<SwerveInput>> sources = new LinkedList<>();
  private final SwerveInput baseSource;

  private SwerveInput currentSource; // store the current source to avoid rebuilding every frame for efficiency

  public SwerveInputManager(SwerveInput baseSource) {
    this.baseSource = baseSource;
    this.currentSource = buildSource();
  }

  public Command runProfile(UnaryOperator<SwerveInput> modifier) {
    return Commands.startEnd(() -> activate(modifier), () -> deactivated(modifier)).ignoringDisable(true);
  }

  private SwerveInput buildSource() {
    SwerveInput source = baseSource;
    for (UnaryOperator<SwerveInput> layer : sources) {
      source = layer.apply(source);
    }
    return source;
  }

  private void activate(UnaryOperator<SwerveInput> source) {
    sources.push(source);
    currentSource = buildSource();
  }

  private void deactivated(UnaryOperator<SwerveInput> source) {
    sources.remove(source);
    currentSource = buildSource();
  }

  public ChassisSpeeds getSpeeds() {
    return currentSource.getSpeeds();
  }
}
