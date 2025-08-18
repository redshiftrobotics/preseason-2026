package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.controllers.HeadingController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceFlipUtil;
import frc.robot.utility.JoystickUtil;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveInputStream {

  private final Drive drive;
  private final HeadingController headingController;

  private Supplier<Translation2d> translationSupplier = () -> Translation2d.kZero;
  private DoubleSupplier rotationSupplier = () -> 0.0;
  private BooleanSupplier fieldRelativeSupplier = () -> true;
  private Supplier<Rotation2d> headingSupplier = () -> Rotation2d.kZero;

  public SwerveInputStream(Drive drive, HeadingController headingController) {
    this.drive = drive;
    this.headingController = headingController;
  }

  public SwerveInputStream translation(Supplier<Translation2d> translationSupplier) {
    this.translationSupplier = translationSupplier;
    return this;
  }

  public SwerveInputStream translationStick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return translation(
        () ->
            getTranslationMetersPerSecond(
                xSupplier.getAsDouble(),
                ySupplier.getAsDouble(),
                drive.getMaxLinearSpeedMetersPerSec()));
  }

  public SwerveInputStream rotation(DoubleSupplier rotationSupplier) {
    this.rotationSupplier = rotationSupplier;
    return this;
  }

  public SwerveInputStream rotationStick(DoubleSupplier omegaSupplier) {
    return rotation(
        () ->
            getOmegaRadiansPerSecond(
                omegaSupplier.getAsDouble(), drive.getMaxAngularSpeedRadPerSec()));
  }

  public SwerveInputStream headingDirection(
      Supplier<Rotation2d> headingAngleSupplier, boolean allianceRelative) {
    headingSupplier =
        allianceRelative
            ? () -> AllianceFlipUtil.apply(headingAngleSupplier.get())
            : headingAngleSupplier;
    rotationSupplier = headingController::calculate;
    return this;
  }

  public SwerveInputStream headingStick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
    return headingDirection(
        () -> getHeadingDirection(xSupplier.getAsDouble(), ySupplier.getAsDouble()), true);
  }

  public SwerveInputStream facingPoint(Pose2d point) {
    return headingDirection(
        () -> {
          var diff = drive.getRobotPose().getTranslation().minus(point.getTranslation());
          return diff.getNorm() > 1e-6 ? diff.getAngle().plus(Rotation2d.k180deg) : null;
        },
        false);
  }

  public SwerveInputStream fieldRelative(Boolean fieldRelative) {
    this.fieldRelativeSupplier = () -> fieldRelative;
    return this;
  }

  public SwerveInputStream fieldRelative(BooleanSupplier fieldRelativeSupplier) {
    this.fieldRelativeSupplier = fieldRelativeSupplier;
    return this;
  }

  public SwerveInputStream translationCoefficient(double coefficient) {
    Supplier<Translation2d> originalTranslationSupplier = translationSupplier;
    translationSupplier = () -> originalTranslationSupplier.get().times(coefficient);
    return this;
  }

  public SwerveInputStream rotationCoefficient(double coefficient) {
    DoubleSupplier originalRotationSupplier = rotationSupplier;
    rotationSupplier = () -> originalRotationSupplier.getAsDouble() * coefficient;
    return this;
  }

  public SwerveInputStream copy() {
    SwerveInputStream copy = new SwerveInputStream(drive, headingController);
    copy.headingSupplier = this.headingSupplier;
    copy.translationSupplier = this.translationSupplier;
    copy.rotationSupplier = this.rotationSupplier;
    copy.fieldRelativeSupplier = this.fieldRelativeSupplier;
    return copy;
  }

  public Command getCommand() {
    return DriveCommands.joystickDrive(
            drive, translationSupplier, rotationSupplier, fieldRelativeSupplier)
        .beforeStarting(
            () -> {
              headingController.reset();
              headingController.setSetpointSupplier(headingSupplier);
            });
  }

  private static Translation2d getTranslationMetersPerSecond(
      double xInput, double yInput, double maxTranslationSpeedMetersPerSecond) {

    Translation2d translation = new Translation2d(xInput, yInput);

    // get length of linear velocity vector, and apply deadband to it for noise reduction
    double magnitude = JoystickUtil.applyDeadband(translation.getNorm());

    if (magnitude == 0) return new Translation2d();

    // squaring the magnitude of the vector makes for quicker ramp up and slower fine control,
    // magnitude should always be positive
    double magnitudeSquared = Math.copySign(Math.pow(magnitude, 2), 1);

    // get a vector with the same angle as the base linear velocity vector but with the
    // magnitude squared
    Translation2d squaredLinearVelocity =
        new Pose2d(new Translation2d(), translation.getAngle())
            .transformBy(new Transform2d(magnitudeSquared, 0.0, new Rotation2d()))
            .getTranslation();

    // return final value
    return squaredLinearVelocity.times(maxTranslationSpeedMetersPerSecond);
  }

  private static double getOmegaRadiansPerSecond(
      double omegaInput, double maxAngularSpeedRadPerSec) {

    // get rotation speed, and apply deadband
    double omega = JoystickUtil.applyDeadband(omegaInput);

    // square the omega value for quicker ramp up and slower fine control
    // make sure to copy the sign over for direction
    double omegaSquared = Math.copySign(Math.pow(omega, 2), omega);

    // return final value
    return omegaSquared * maxAngularSpeedRadPerSec;
  }

  private static Rotation2d getHeadingDirection(double xInput, double yInput) {
    // Get desired angle as a vector
    final Translation2d desiredAngle = new Translation2d(xInput, yInput);

    // If the vector length is longer then our deadband update the heading controller
    if (desiredAngle.getNorm() > 0.5) {
      return desiredAngle.getAngle();
    }

    return null;
  }
}
