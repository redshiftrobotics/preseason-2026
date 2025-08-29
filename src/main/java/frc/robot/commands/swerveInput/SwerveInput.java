package frc.robot.commands.swerveInput;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.controllers.HeadingController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceFlipUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveInput {

  private final Drive drive;

  private final Supplier<Translation2d> translationSupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier fieldRelativeSupplier;

  public final String name;

  public SwerveInput(Drive drive, String name) {
    this(drive, () -> Translation2d.kZero, () -> 0.0, () -> true, name);
  }

  public SwerveInput(
      Drive drive,
      Supplier<Translation2d> translationSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldRelativeSupplier,
      String name) {
    this.drive = drive;
    this.translationSupplier = translationSupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldRelativeSupplier = fieldRelativeSupplier;
    this.name = name;
  }

  public ChassisSpeeds getSpeeds() {
    Translation2d translation = translationSupplier.get();
    double rotation = rotationSupplier.getAsDouble();
    boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();

    ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    if (fieldRelative) {
      speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drive.getRobotPose().getRotation());
    }

    return speeds;
  }

  public SwerveInput withTranslation(Supplier<Translation2d> translationSupplier) {
    return new SwerveInput(
        drive, translationSupplier, rotationSupplier, fieldRelativeSupplier, name);
  }

  public SwerveInput withTranslationStick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return withTranslation(
        () ->
            SwerveJoystickUtil.getTranslationMetersPerSecond(
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                drive.getMaxLinearSpeedMetersPerSec()));
  }

  public SwerveInput withRotation(DoubleSupplier rotationSupplier) {
    return new SwerveInput(
        drive, translationSupplier, rotationSupplier, fieldRelativeSupplier, name);
  }

  public SwerveInput withRotationStick(DoubleSupplier omegaSupplier) {
    return withRotation(
        () ->
            SwerveJoystickUtil.getOmegaRadiansPerSecond(
                omegaSupplier.getAsDouble(), drive.getMaxAngularSpeedRadPerSec()));
  }

  public SwerveInput withHeadingDirection(Supplier<Rotation2d> headingAngleSupplier) {
    HeadingController headingController = new HeadingController(drive, headingAngleSupplier);

    return new SwerveInput(
        drive, translationSupplier, headingController::calculate, fieldRelativeSupplier, name);
  }

  public SwerveInput withHeadingDirection(
      Supplier<Rotation2d> headingAngleSupplier, boolean allianceRelative) {
    return withHeadingDirection(
        () -> {
          Rotation2d angle = headingAngleSupplier.get();
          if (angle == null || !allianceRelative) return angle;
          return AllianceFlipUtil.apply(angle);
        });
  }

  public SwerveInput withHeadingStick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return withHeadingDirection(
        () ->
            SwerveJoystickUtil.getHeadingDirection(
                xSupplier.getAsDouble(), ySupplier.getAsDouble()),
        true);
  }

  public SwerveInput facingPoint(Translation2d point) {
    return withHeadingDirection(
        () -> {
          var diff = drive.getRobotPose().getTranslation().minus(point);
          return diff.getNorm() > 1e-6 ? diff.getAngle().plus(Rotation2d.k180deg) : null;
        });
  }

  public SwerveInput withFieldRelativeEnabled(boolean fieldRelative) {
    return new SwerveInput(drive, translationSupplier, rotationSupplier, () -> fieldRelative, name);
  }

  public SwerveInput withName(String name) {
    return new SwerveInput(
        drive, translationSupplier, rotationSupplier, fieldRelativeSupplier, name);
  }

  public SwerveInput appendName(String name) {
    if (this.name == null) {
      return withName(name);
    }
    return withName(name + " > " + this.name);
  }
}
