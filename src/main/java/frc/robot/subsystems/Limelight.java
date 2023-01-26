package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.sensors.LimelightHelper;

public class Limelight extends LimelightHelper {
    private static final double CONE_WIDTH = 0.21,
                                CONE_HEIGHT = 0.32,
                                CUBE_WIDTH = 0;

    private static final int APRILTAG_PIPELINE = 0,
                             CONE_PIPELINE = 1,
                             CUBE_PIPELINE = 2;

    public Limelight(String networktablesName) {
        super(networktablesName);
    }

    public Translation2d getConePoseRelativeToCamera(){
        setPipeline(CONE_PIPELINE);
        Pose2d test = new Pose2d();
        var boundingBox = getBoundingBox();
        double distance = getDistance(CONE_HEIGHT, CONE_WIDTH, boundingBox);
        double angle = Units.degreesToRadians(getTargetX()); //TODO is it x or y
        return new Translation2d(distance * Math.cos(angle), distance * Math.sin(angle)); //TODO coordinate systems
    }

    public Translation2d getCubePoseRelativeToCamera(){
        setPipeline(CUBE_PIPELINE);
        var boundingBox = getBoundingBox();
        double distance = getDistance(CUBE_WIDTH, CUBE_WIDTH, boundingBox);
        double angle = getBoundingBox().getCenter().getX();
        return new Translation2d(distance * Math.cos(angle), distance * Math.sin(angle));
    }
}
