package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.List;

/**
 * Contains various field dimensions and useful reference points. Dimensions are in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in Meters</b>
 *
 * <p>All translations and poses are stored with the origin at the rightmost point on the BLUE
 * ALLIANCE wall.
 *
 * <p>Length refers to the <i>x</i> direction. Width refers to the <i>y</i> direction. (as described
 * by WPILib)
 *
 * @see https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html
 */
public class FieldConstants {

  public static final AprilTagFieldLayout FIELD_APRIL_TAGS =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final AprilTagFieldLayout FIELD_NO_APRIL_TAGS =
      new AprilTagFieldLayout(
          List.of(), FIELD_APRIL_TAGS.getFieldLength(), FIELD_APRIL_TAGS.getFieldWidth());

  public static final Translation2d FIELD_CORNER_TO_CORNER =
      new Translation2d(FIELD_APRIL_TAGS.getFieldLength(), FIELD_APRIL_TAGS.getFieldWidth());
}
