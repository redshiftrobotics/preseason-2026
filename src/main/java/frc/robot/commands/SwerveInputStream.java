package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.controllers.HeadingController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utility.AllianceFlipUtil;
import frc.robot.utility.JoystickUtil;

import java.util.Deque;
import java.util.LinkedList;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SwerveInputStream {

  private final Drive drive;
  private final HeadingController headingController;

  private final Deque<SwerveInputSource> sourcesStack = new LinkedList<>();
  private final SwerveInputSource defaultSource = new SwerveInputSource()
      .withTranslation(() -> Translation2d.kZero)
      .withRotation(() -> 0.0)
      .withHeadingDirection(() -> null, false)
      .withFieldRelativeEnabled(() -> true);

  private SwerveInputSource currentSource = defaultSource;

  public SwerveInputStream(Drive drive, HeadingController headingController) {
    this.drive = drive;
    this.headingController = headingController;
  }

  public SwerveInputSource getDefaultSource() {
    return defaultSource;
  }

  public SwerveInputSource newSource() {
    return new SwerveInputSource();
  }

  private SwerveInputSource buildSource() {
    SwerveInputSource source = defaultSource.copy();
    for (SwerveInputSource s : sourcesStack) {
      source.withExtension(s);
    }
    return source;
  }

  private void push(SwerveInputSource source) {
    sourcesStack.push(source);
    currentSource = buildSource(); 
    headingController.reset();
    headingController.setSetpointSupplier(currentSource.headingSupplier);
  }

  private void remove(SwerveInputSource source) {
    sourcesStack.remove(source);
    currentSource = buildSource(); 
  }

  public Command getCommand() {
    return DriveCommands.drive(
            drive, currentSource.translationSupplier, currentSource.rotationSupplier, currentSource.fieldRelativeSupplier); 
  }

  public class SwerveInputSource {
    public String name;
    public Supplier<Translation2d> translationSupplier;
    public DoubleSupplier rotationSupplier;
    public Supplier<Rotation2d> headingSupplier;
    public BooleanSupplier fieldRelativeSupplier;

    public SwerveInputSource copy() {
      SwerveInputSource copy = new SwerveInputSource();
      copy.name = this.name;
      copy.translationSupplier = this.translationSupplier;
      copy.rotationSupplier = this.rotationSupplier;
      copy.headingSupplier = this.headingSupplier;
      copy.fieldRelativeSupplier = this.fieldRelativeSupplier;
      return copy;
    }

    public Command getCommand() {
      return Commands.runEnd(
        () -> push(this),
        () -> remove(this)
      );
    }

    public Command getCommand(String name) {
      return withName(name).getCommand();
    }
    
    public SwerveInputSource withTranslation(Supplier<Translation2d> translationSupplier) {
      this.translationSupplier = translationSupplier;
      return this;
    }

    public SwerveInputSource withTranslationStick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
      return withTranslation(
          () ->
              getTranslationMetersPerSecond(
                  xSupplier.getAsDouble(),
                  ySupplier.getAsDouble(),
                  drive.getMaxLinearSpeedMetersPerSec()));
    }

    public SwerveInputSource withRotation(DoubleSupplier rotationSupplier) {
      this.rotationSupplier = rotationSupplier;
      return this;
    }

    public SwerveInputSource withRotationStick(DoubleSupplier omegaSupplier) {
      return withRotation(
          () ->
              getOmegaRadiansPerSecond(
                  omegaSupplier.getAsDouble(), drive.getMaxAngularSpeedRadPerSec()));
    }

    public SwerveInputSource withHeadingDirection(
        Supplier<Rotation2d> headingAngleSupplier, boolean allianceRelative) {
      headingSupplier = () -> {
        var angle = headingAngleSupplier.get();
        if (angle == null || !allianceRelative) return angle;
        return AllianceFlipUtil.apply(angle);
      };
      rotationSupplier = headingController::calculate;
      return this;
    }

    public SwerveInputSource withHeadingStick(DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
      return withHeadingDirection(
          () -> getHeadingDirection(xSupplier.getAsDouble(), ySupplier.getAsDouble()), true);
    }

    public SwerveInputSource withTargetPoint(Pose2d point) {
      return withHeadingDirection(
          () -> {
            var diff = drive.getRobotPose().getTranslation().minus(point.getTranslation());
            return diff.getNorm() > 1e-6 ? diff.getAngle().plus(Rotation2d.k180deg) : null;
          },
          false);
    }

    public SwerveInputSource withFieldRelativeEnabled(BooleanSupplier fieldRelative) {
      this.fieldRelativeSupplier = fieldRelative;
      return this;
    }

    public SwerveInputSource withName(String name) {
      this.name = name;
      return this;
    }

    public SwerveInputSource withExtension(SwerveInputSource other) {
      if (other.name != null) this.name = (this.name != null ? this.name + " + " : "") + other.name;
      if (other.translationSupplier != null) this.translationSupplier = other.translationSupplier;
      if (other.rotationSupplier != null) this.rotationSupplier = other.rotationSupplier;
      if (other.headingSupplier != null) this.headingSupplier = other.headingSupplier;
      if (other.fieldRelativeSupplier != null) this.fieldRelativeSupplier = other.fieldRelativeSupplier;
      return this;
    }
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