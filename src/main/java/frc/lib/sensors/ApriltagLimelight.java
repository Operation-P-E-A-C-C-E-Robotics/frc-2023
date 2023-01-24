package frc.lib.sensors;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveAccelerationLimiter;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;

public class ApriltagLimelight {
    private final static String NETWORKTABLES_NAME = "limelight"; //TODO
    private final DoubleArraySubscriber botposeSub;
    private DoubleArraySubscriber cameraPoseSub;
    private DoubleSubscriber latencySub;
    private DoubleSubscriber tLong;
    private DoubleArraySubscriber tCorners;
    

    

    public ApriltagLimelight(){
        NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
        NetworkTable limelight = networkTables.getTable(NETWORKTABLES_NAME);
        DoubleArrayTopic botposeTopic = limelight.getDoubleArrayTopic("botpose");
        DoubleArrayTopic cameraPoseTopic = limelight.getDoubleArrayTopic("campose");
        botposeSub = botposeTopic.subscribe(new double[0]);
        latencySub = limelight.getDoubleTopic("tl").subscribe(0);
        tCorners = limelight.getDoubleArrayTopic("tcornxy").subscribe(new double[0]);
        tLong = limelight.getDoubleTopic("tlong").subscribe(0);
        cameraPoseSub = cameraPoseTopic.subscribe(new double[0]);
    }

    public double getLatency(){
        return latencySub.get() / 1000;
    }

    double halfFieldWidth = 16.48/2;
    double halfFieldHeight = 8.1/2;
    public void updatePoseEstimator(DifferentialDrivePoseEstimator estimator){
        TimestampedDoubleArray[] visionMeasurements = botposeSub.readQueue();
        for(TimestampedDoubleArray i : visionMeasurements){
            double time = Timer.getFPGATimestamp();// - getLatency();
            System.out.println("asdfasdf");
            System.out.println(Timer.getFPGATimestamp());
            System.out.println(time);
            double[] val = i.value;
            if(val.length == 6) {
                estimator.addVisionMeasurement(new Pose2d(
                        val[0] + halfFieldWidth, //TODO these indices are probably wrong
                        val[1] + halfFieldHeight,
                        Rotation2d.fromDegrees(val[5])
                )//.relativeTo(new Pose2d(halfFieldWidth, halfFieldHeight, Rotation2d.fromDegrees(180)))
                ,time,
                VecBuilder.fill(1/getTLONGGGGGGG(), 1/getTLONGGGGGGG(), 10/getTLONGGGGGGG()));
            }
        }
    }

    // public Translation2d getBoundingBox(){
    //     new Translation2d().
    // }

    public double getTLONGGGGGGG(){
        return tLong.get();
    }

    public Pose2d getCameraPose() {
        var camData = cameraPoseSub.get();
        if (camData.length == 3) {
            return new Pose2d(camData[0], camData[1], new Rotation2d());
        }
        return new Pose2d();
    }
}
