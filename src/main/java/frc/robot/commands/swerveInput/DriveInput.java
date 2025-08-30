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

/**
 * A class that encapsulates the inputs for driving a swerve drive robot. It provides methods to
 * modify the translation, rotation, and field-relative settings.
 */
public class DriveInput {

  private final Drive drive;

  private final List<String> labels;

  private final Supplier<Translation2d> translationSupplier;
  private final DoubleSupplier rotationSupplier;
  private final BooleanSupplier fieldRelativeSupplier;

  public DriveInput(Drive drive, String label) {
    this(drive, List.of(label), () -> Translation2d.kZero, () -> 0.0, () -> true);
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

  public ChassisSpeeds getChassisSpeeds() {
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

  public DriveInput translation(Supplier<Translation2d> translationSupplier) {
    return new DriveInput(
        drive, labels, translationSupplier, rotationSupplier, fieldRelativeSupplier);
  }

  public DriveInput translationStick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return translation(
        () ->
            SwerveJoystickUtil.getTranslationMetersPerSecond(
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                drive.getMaxLinearSpeedMetersPerSec()));
  }

  public DriveInput rotation(DoubleSupplier rotationSupplier) {
    return new DriveInput(
        drive, labels, translationSupplier, rotationSupplier, fieldRelativeSupplier);
  }

  public DriveInput rotationStick(DoubleSupplier omegaSupplier) {
    return rotation(
        () ->
            SwerveJoystickUtil.getOmegaRadiansPerSecond(
                omegaSupplier.getAsDouble(), drive.getMaxAngularSpeedRadPerSec()));
  }

  public DriveInput headingDirection(Supplier<Rotation2d> headingAngleSupplier) {
    HeadingController headingController = new HeadingController(drive, headingAngleSupplier);

    return new DriveInput(
        drive, labels, translationSupplier, headingController::calculate, fieldRelativeSupplier);
  }

  public DriveInput headingDirection(
      Supplier<Rotation2d> headingAngleSupplier, boolean allianceRelative) {
    return headingDirection(
        () -> {
          Rotation2d angle = headingAngleSupplier.get();
          if (angle == null || !allianceRelative) return angle;
          return AllianceFlipUtil.apply(angle);
        });
  }

  public DriveInput headingStick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return headingDirection(
        () ->
            SwerveJoystickUtil.getHeadingDirection(
                xSupplier.getAsDouble(), ySupplier.getAsDouble()),
        true);
  }

  public DriveInput facingPoint(Translation2d point) {
    return headingDirection(
        () -> {
          var diff = drive.getRobotPose().getTranslation().minus(point);
          return diff.getNorm() > 1e-6 ? diff.getAngle().plus(Rotation2d.k180deg) : null;
        });
  }

  public DriveInput fieldRelativeEnabled() {
    return new DriveInput(drive, labels, translationSupplier, rotationSupplier, () -> true);
  }

  public DriveInput fieldRelativeDisabled() {
    return new DriveInput(drive, labels, translationSupplier, rotationSupplier, () -> false);
  }

  public DriveInput translationCoefficient(double coefficient) {
    return translation(() -> translationSupplier.get().times(coefficient));
  }

  public DriveInput rotationCoefficient(double coefficient) {
    return rotation(() -> rotationSupplier.getAsDouble() * coefficient);
  }

  public DriveInput addLabel(String label) {
    return new DriveInput(
        drive,
        Stream.concat(labels.stream(), Stream.of(label)).toList(),
        translationSupplier,
        rotationSupplier,
        fieldRelativeSupplier);
  }

  public List<String> getLabels() {
    return labels;
  }
}
