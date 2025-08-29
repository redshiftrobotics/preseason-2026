package frc.robot.commands.swerveInput;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.controllers.HeadingController;
import frc.robot.subsystems.drive.Drive;
import java.util.Deque;
import java.util.LinkedList;
import java.util.function.Consumer;

public class SwerveInputManager {

  private final Drive drive;
  private final HeadingController headingController;

  private final Deque<Consumer<SwerveInputLayer>> sourcesStack = new LinkedList<>();
  private final SwerveInputLayer defaultSource;

  private SwerveInputLayer currentSource;

  public SwerveInputManager(Drive drive, HeadingController headingController) {
    this.drive = drive;
    this.headingController = headingController;

    defaultSource = new SwerveInputLayer(drive, headingController);

    defaultDriveWith(
        input ->
            input
                .withTranslation(() -> Translation2d.kZero)
                .withRotation(() -> 0.0)
                .withHeadingDirection(() -> null, false)
                .withFieldRelativeEnabled(true));
  }

  public Command driveWith(Consumer<SwerveInputLayer> layer) {
    return Commands.startEnd(() -> activate(layer), () -> deactivated(layer)).ignoringDisable(true);
  }

  public void defaultDriveWith(Consumer<SwerveInputLayer> layer) {
    layer.accept(defaultSource);
    handleNewSources();
  }

  public Command getCommand() {
    Command driveCommand =
        DriveCommands.drive(drive, this::getTranslation, this::getRotation, this::getFieldRelative);
    return driveCommand.alongWith(Commands.run(() -> driveCommand.setName(getCurrentSourceName())));
  }

  private SwerveInputLayer buildSource() {
    SwerveInputLayer source = defaultSource.copy();
    for (Consumer<SwerveInputLayer> layer : sourcesStack) {
      layer.accept(source);
    }
    return source;
  }

  private void activate(Consumer<SwerveInputLayer> source) {
    sourcesStack.push(source);
    handleNewSources();
  }

  private void deactivated(Consumer<SwerveInputLayer> source) {
    sourcesStack.remove(source);
    handleNewSources();
  }

  private void handleNewSources() {
    currentSource = buildSource();
    headingController.reset();
    headingController.setSetpointSupplier(currentSource.headingSupplier);
  }

  private Translation2d getTranslation() {
    return currentSource.translationSupplier.get();
  }

  private double getRotation() {
    return currentSource.rotationSupplier.getAsDouble();
  }

  private boolean getFieldRelative() {
    return currentSource.fieldRelativeSupplier.getAsBoolean();
  }

  private String getCurrentSourceName() {
    return currentSource.name;
  }
}
