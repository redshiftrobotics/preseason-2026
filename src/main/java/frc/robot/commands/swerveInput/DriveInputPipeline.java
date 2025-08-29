package frc.robot.commands.swerveInput;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.LinkedList;
import java.util.List;
import java.util.function.UnaryOperator;

/**
 * A pipeline for composing multiple layers of {@link DriveInput}. Each layer is a function that
 * takes a {@link DriveInput} and returns a modified {@link DriveInput}. Layers can be activated
 * and deactivated, and the final {@link DriveInput} is the result of applying all active layers in
 * sequence to the base {@link DriveInput}.
 */
public class DriveInputPipeline {

  private final List<UnaryOperator<DriveInput>> modifiers = new LinkedList<>();
  private final DriveInput baseInput;

  private DriveInput composedInput; // store to avoid rebuilding every frame for efficiency

  public DriveInputPipeline(DriveInput baseInput) {
    this.baseInput = baseInput;
    this.composedInput = compose();
  }

  public Command activateLayer(UnaryOperator<DriveInput> modifier) {
    return Commands.startEnd(() -> activate(modifier), () -> deactivate(modifier))
        .ignoringDisable(true);
  }

  public void clearLayers() {
    modifiers.clear();
    composedInput = compose();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return composedInput.getSpeeds();
  }

  public List<String> getActiveLayers() {
    return composedInput.getLabels();
  }

  private void activate(UnaryOperator<DriveInput> layer) {
    modifiers.add(layer);
    composedInput = compose();
  }

  private void deactivate(UnaryOperator<DriveInput> layer) {
    modifiers.remove(layer);
    composedInput = compose();
  }

  private DriveInput compose() {
    DriveInput input = baseInput;
    for (UnaryOperator<DriveInput> layer : modifiers) {
      input = layer.apply(input);
    }
    return input;
  }

}
