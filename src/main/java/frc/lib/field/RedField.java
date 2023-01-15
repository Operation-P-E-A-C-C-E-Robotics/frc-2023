package frc.lib.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

//TODO
/**
 * keep track of field relative coordinates for various points of interest
 * coordinate system follows WPILib standard:
 * (0,0) falls on the bottom left corner of the field when your alliance side
 *      is on the left
 * The Y axis is along your alliance wall
 * The X axis points away from the alliance wall
 */
public class RedField {
    public static final Translation2d[] TOP_CONE_SCORE_LOCATIONS = {
        new Translation2d(1, 2),
        new Translation2d(0,-14)
    };

    public static final Translation2d[] MID_CONE_SCORE_LOCATIONS = {
        new Translation2d(0, 0)
    };

    public static final Translation2d[] TOP_SPHEQUARE_SCORE_LOCATIONS = {
        new Translation2d(0,0)
    };

    public static final Translation2d[] MID_SPHEQUARE_SCORE_LOCATIONS = {
        new Translation2d(0,0)
    };

    public static final Translation2d[] JOINT_SCORE_LOCATIONS = {
        new Translation2d(0,0)
    };

    public static final Translation2d CHARGE_STATION = new Translation2d(0,0);

    public static final Translation2d[] SUBSTATION_SHELVES = {
        new Translation2d(0,0)
    };

    /**
     * NOT DONE determine which side of the field the robot is on
     * @param translation the robot's translation
     * @return true if the robot is on your alliance color side.
     */
    public static boolean onAllianceFieldSide(Translation2d translation){
        return translation.getX() < 5; //TODO arbitrary number
    }

    /**
     * figure out which translation is closest to the robot from a list of translations
     * @param options array of translations to choose from
     * @param translation The robot's translation
     * @return the translation that is nearest to the robot
     */
    public static Translation2d nearestLocation(Translation2d[] options, Translation2d translation){
        double min_distance = 100;
        Translation2d min = options[0];
        for (Translation2d i : options) {
            var dist = translation.getDistance(i);
            if(dist < min_distance) {
                min = i;
                min_distance = dist;
            }
        }
        return min;
    }

    /**
    * convert a field-relative coordinate to a robot-relative coordinate
    * @param translation the point to convert
    * @param robot the {@link Pose2d} of the robot
    * @return the {@link Translation2d} converted to robot-relative coordinates.
    */
    public static Translation2d makeRobotRelative(Translation2d translation, Pose2d robot){
        var relative = translation.minus(robot.getTranslation());
        relative = translation.rotateBy(robot.getRotation());
        return relative;
    }
}
