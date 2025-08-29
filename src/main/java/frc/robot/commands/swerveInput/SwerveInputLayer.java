package frc.robot.commands.swerveInput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.controllers.HeadingController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceFlipUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveInputLayer {

  private final Drive drive;
  private final HeadingController headingController;

  public String name;
  public Supplier<Translation2d> translationSupplier;
  public DoubleSupplier rotationSupplier;
  public Supplier<Rotation2d> headingSupplier;
  public BooleanSupplier fieldRelativeSupplier;

  public SwerveInputLayer(Drive drive, HeadingController headingController) {
    this.drive = drive;
    this.headingController = headingController;
  }

  public SwerveInputLayer copy() {
    SwerveInputLayer copy = new SwerveInputLayer(drive, headingController);
    copy.name = this.name;
    copy.translationSupplier = this.translationSupplier;
    copy.rotationSupplier = this.rotationSupplier;
    copy.headingSupplier = this.headingSupplier;
    copy.fieldRelativeSupplier = this.fieldRelativeSupplier;
    return copy;
  }

  public Command getCommand() {
    return DriveCommands.drive(drive, translationSupplier, rotationSupplier, fieldRelativeSupplier)
        .beforeStarting(
            () -> {
              headingController.reset();
              headingController.setSetpointSupplier(headingSupplier);
            });
  }

  public Command getCommand(String name) {
    return named(name).getCommand();
  }

  public SwerveInputLayer withTranslation(Supplier<Translation2d> translationSupplier) {
    this.translationSupplier = translationSupplier;
    return this;
  }

  public SwerveInputLayer withTranslationStick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return withTranslation(
        () ->
            SwerveJoystickUtil.getTranslationMetersPerSecond(
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                drive.getMaxLinearSpeedMetersPerSec()));
  }

  public SwerveInputLayer withRotation(DoubleSupplier rotationSupplier) {
    this.rotationSupplier = rotationSupplier;
    return this;
  }

  public SwerveInputLayer withRotationStick(DoubleSupplier omegaSupplier) {
    return withRotation(
        () ->
            SwerveJoystickUtil.getOmegaRadiansPerSecond(
                omegaSupplier.getAsDouble(), drive.getMaxAngularSpeedRadPerSec()));
  }

  public SwerveInputLayer withHeadingDirection(
      Supplier<Rotation2d> headingAngleSupplier, boolean allianceRelative) {
    headingSupplier =
        () -> {
          var angle = headingAngleSupplier.get();
          if (angle == null || !allianceRelative) return angle;
          return AllianceFlipUtil.apply(angle);
        };
    rotationSupplier = headingController::calculate;
    return this;
  }

  public SwerveInputLayer withHeadingStick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return withHeadingDirection(
        () ->
            SwerveJoystickUtil.getHeadingDirection(
                xSupplier.getAsDouble(), ySupplier.getAsDouble()),
        true);
  }

  public SwerveInputLayer withTargetPoint(Pose2d point) {
    return withHeadingDirection(
        () -> {
          var diff = drive.getRobotPose().getTranslation().minus(point.getTranslation());
          return diff.getNorm() > 1e-6 ? diff.getAngle().plus(Rotation2d.k180deg) : null;
        },
        false);
  }

  public SwerveInputLayer withFieldRelativeEnabled(boolean fieldRelative) {
    this.fieldRelativeSupplier = () -> fieldRelative;
    return this;
  }

  public SwerveInputLayer withName(String name) {
    this.name = name;
    return this;
  }

  public SwerveInputLayer named(String name) {
    if (this.name == null) {
      return withName(name);
    }
    return withName(name + " | " + this.name);
  }

  public SwerveInputLayer withExtension(SwerveInputLayer other) {
    if (other.name != null) this.name = (this.name != null ? this.name + " + " : "") + other.name;
    if (other.translationSupplier != null) this.translationSupplier = other.translationSupplier;
    if (other.rotationSupplier != null) this.rotationSupplier = other.rotationSupplier;
    if (other.headingSupplier != null) this.headingSupplier = other.headingSupplier;
    if (other.fieldRelativeSupplier != null)
      this.fieldRelativeSupplier = other.fieldRelativeSupplier;
    return this;
  }
}
