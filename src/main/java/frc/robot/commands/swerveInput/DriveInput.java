package frc.robot.commands.swerveInput;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.controllers.HeadingController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceFlipUtil;

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

public class DriveInput {

  private final Drive drive;

  private final List<String> labels;

  private final Supplier<Translation2d> translationSupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier fieldRelativeSupplier;


  public DriveInput(Drive drive, String name) {
    this(drive, List.of(name), () -> Translation2d.kZero, () -> 0.0, () -> true);
  }

  public DriveInput(
      Drive drive,
      List<String> labels,
      Supplier<Translation2d> translationSupplier,
      DoubleSupplier rotationSupplier,
      BooleanSupplier fieldRelativeSupplier) {
    this.drive = drive;
    this.translationSupplier = translationSupplier;
    this.rotationSupplier = rotationSupplier;
    this.fieldRelativeSupplier = fieldRelativeSupplier;
    this.labels = labels;
  }

  public ChassisSpeeds getSpeeds() {
    Translation2d translation = translationSupplier.get();
    double rotation = rotationSupplier.getAsDouble();
    boolean fieldRelative = fieldRelativeSupplier.getAsBoolean();

    ChassisSpeeds speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
    if (fieldRelative) {
      speeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              speeds, AllianceFlipUtil.apply(drive.getRobotPose().getRotation()));
    }

    return speeds;
  }

  public DriveInput withTranslation(Supplier<Translation2d> translationSupplier) {
    return new DriveInput(
        drive, labels, translationSupplier, rotationSupplier, fieldRelativeSupplier);
  }

  public DriveInput withTranslationStick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return withTranslation(
        () ->
            SwerveJoystickUtil.getTranslationMetersPerSecond(
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                drive.getMaxLinearSpeedMetersPerSec()));
  }

  public DriveInput withRotation(DoubleSupplier rotationSupplier) {
    return new DriveInput(
        drive, labels, translationSupplier, rotationSupplier, fieldRelativeSupplier);
  }

  public DriveInput withRotationStick(DoubleSupplier omegaSupplier) {
    return withRotation(
        () ->
            SwerveJoystickUtil.getOmegaRadiansPerSecond(
                omegaSupplier.getAsDouble(), drive.getMaxAngularSpeedRadPerSec()));
  }

  public DriveInput withHeadingDirection(Supplier<Rotation2d> headingAngleSupplier) {
    HeadingController headingController = new HeadingController(drive, headingAngleSupplier);

    return new DriveInput(
        drive, labels, translationSupplier, headingController::calculate, fieldRelativeSupplier);
  }

  public DriveInput withHeadingDirection(
      Supplier<Rotation2d> headingAngleSupplier, boolean allianceRelative) {
    return withHeadingDirection(
        () -> {
          Rotation2d angle = headingAngleSupplier.get();
          if (angle == null || !allianceRelative) return angle;
          return AllianceFlipUtil.apply(angle);
        });
  }

  public DriveInput withHeadingStick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return withHeadingDirection(
        () ->
            SwerveJoystickUtil.getHeadingDirection(
                xSupplier.getAsDouble(), ySupplier.getAsDouble()),
        true);
  }

  public DriveInput facingPoint(Translation2d point) {
    return withHeadingDirection(
        () -> {
          var diff = drive.getRobotPose().getTranslation().minus(point);
          return diff.getNorm() > 1e-6 ? diff.getAngle().plus(Rotation2d.k180deg) : null;
        });
  }

  public DriveInput withFieldRelativeEnabled(boolean fieldRelative) {
    return new DriveInput(drive, labels, translationSupplier, rotationSupplier, () -> fieldRelative);
  }

  public DriveInput pushLabel(String label) {
    return new DriveInput(
        drive, Stream.concat(labels.stream(), Stream.of(label)).toList(), translationSupplier, rotationSupplier, fieldRelativeSupplier);
  }

  public List<String> getLabels() {
    return labels;
  }
}
