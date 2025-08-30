package frc.robot.commands.pipeline;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utility.JoystickUtil;

public class SwerveJoystickUtil {
  public static Translation2d getTranslationMetersPerSecond(
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

  public static double getOmegaRadiansPerSecond(
      double omegaInput, double maxAngularSpeedRadPerSec) {

    // get rotation speed, and apply deadband
    double omega = JoystickUtil.applyDeadband(omegaInput);

    // square the omega value for quicker ramp up and slower fine control
    // make sure to copy the sign over for direction
    double omegaSquared = Math.copySign(Math.pow(omega, 2), omega);

    // return final value
    return omegaSquared * maxAngularSpeedRadPerSec;
  }

  public static Rotation2d getHeadingDirection(double xInput, double yInput) {
    // Get desired angle as a vector
    final Translation2d desiredAngle = new Translation2d(xInput, yInput);

    // If the vector length is longer then our deadband update the heading controller
    if (desiredAngle.getNorm() > 0.5) {
      return desiredAngle.getAngle();
    }

    return null;
  }
}
