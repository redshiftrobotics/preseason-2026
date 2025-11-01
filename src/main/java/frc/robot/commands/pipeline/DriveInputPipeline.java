package frc.robot.commands.pipeline;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.LinkedList;
import java.util.List;
import java.util.function.UnaryOperator;

/**
 * A pipeline for modifying {@link DriveInput} using layers of {@link UnaryOperator}. Each layer can
 * modify the input, and layers can be activated or deactivated.
 *
 * <p>The pipeline starts with a base {@link DriveInput}, and each active layer is applied in the
 * order they were activated, with the most recently activated layer being applied last.
 *
 * <p>This allows for multiple modifications to be applied to the drive input in a modular way, such
 * as adding a slow mode layer, combined with an aiming layer, without running into the issue of
 * commands wanting exclusive control over the drive input.
 */
public class DriveInputPipeline {

  private final DriveInput baseInput;
  private final List<UnaryOperator<DriveInput>> layers = new LinkedList<>();

  private DriveInput composedInput; // store to avoid rebuilding every frame for efficiency

  /**
   * Creates a new {@link DriveInputPipeline} with the given base {@link DriveInput}.
   *
   * @param baseInput The base {@link DriveInput} that subsequent layers will modify.
   */
  public DriveInputPipeline(DriveInput baseInput) {
    this.baseInput = baseInput;
    this.composedInput = compose();
  }

  /**
   * Activates a layer for the duration of the returned command. The layer will be deactivated when
   * the command ends.
   *
   * <p>When the layer is activated, it is applied to the base input after all previously activated
   * layers.
   *
   * @param layer A function that returns a new {@link DriveInput} with additional behavior.
   * @return A command that activates the layer while it is running.
   */
  public Command activateLayer(UnaryOperator<DriveInput> layer) {
    return Commands.startEnd(() -> activate(layer), () -> deactivate(layer)).ignoringDisable(true);
  }

  /** Clears all active modifying layers. */
  public void clearLayers() {
    layers.clear();
    composedInput = compose();
  }

  /**
   * Gets the current {@link ChassisSpeeds} output of the pipeline.
   *
   * <p>The speed comes from the base input modified by all active layers.
   *
   * @return The current {@link ChassisSpeeds}.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return composedInput.getChassisSpeeds();
  }

  /**
   * Gets all active layers' labels, with the most recently activated (and last applied) layer at
   * the end of the list.
   *
   * <p>This can be used for debugging or displaying the current state of the pipeline.
   *
   * @return A list of labels of all active layers.
   */
  public List<String> getActiveLayers() {
    return composedInput.getLabels();
  }

  private void activate(UnaryOperator<DriveInput> layer) {
    if (layers.add(layer)) {
      composedInput = compose();
    }
  }

  private void deactivate(UnaryOperator<DriveInput> layer) {
    if (layers.remove(layer)) {
      composedInput = compose();
    }
  }

  private DriveInput compose() {
    DriveInput input = baseInput;
    for (UnaryOperator<DriveInput> layer : layers) {
      input = layer.apply(input);
    }
    return input;
  }
}
