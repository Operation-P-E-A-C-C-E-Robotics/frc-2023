package frc.lib.sensors;

import edu.wpi.first.math.controller.DifferentialDriveAccelerationLimiter;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.*;

public class ApriltagLimelight {
    private final static String NETWORKTABLES_NAME = "limelight"; //TODO
    private final DoubleArraySubscriber botposeSub;

    public ApriltagLimelight(){
        NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
        NetworkTable limelight = networkTables.getTable(NETWORKTABLES_NAME);
        DoubleArrayTopic botposeTopic = limelight.getDoubleArrayTopic("botpose");
        botposeSub = botposeTopic.subscribe(new double[0]);
    }

    public void updatePoseEstimator(DifferentialDrivePoseEstimator estimator){
        TimestampedDoubleArray[] visionMeasurements = botposeSub.readQueue();
        for(TimestampedDoubleArray i : visionMeasurements){
            System.out.println(i);
            double time = i.serverTime;
            double[] val = i.value;
            System.out.println(val.length);
            if(val.length == 6) {
                estimator.addVisionMeasurement(new Pose2d(
                        val[0], //TODO these indices are probably wrong
                        val[1],
                        new Rotation2d(val[3])
                ),time); //TODO varying std deviations based on distance from tags
            }
        }
    }
}
