package frc.lib.sensors;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Timer;

public class LimelightHelper {
    private static final double FOCAL_LENGTH = (1 * 240) / 0.32; //TODO 183 px = 0.21 meters

    private final DoubleArraySubscriber botpose, campose, camtran, tcornxy, tc;
    private final DoubleSubscriber tv, tx, ty, ta, ts, tl, tshort, tlong, thor, tvert;
    private final IntegerSubscriber getpipe, tid;
    private final IntegerPublisher ledMode, camMode, stream, snapshot, pipeline;
    private final DoubleArrayPublisher crop;


    /**
     * TODO only initialize variables that are used, and cache values to avoid unnecessary resource usage
     * a nice limelight helper for color object and
     * apriltags
     * @param networktablesName the limelight's table in networktables
     */
    public LimelightHelper(String networktablesName){
        NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
        NetworkTable limelight = networkTables.getTable(networktablesName);

        tv = limelight.getDoubleTopic("tv").subscribe(0); //has target 0 or 1
        tx = limelight.getDoubleTopic("tx").subscribe(0); //crosshair x degrees
        ty = limelight.getDoubleTopic("ty").subscribe(0); //crosshair y degrees
        ta = limelight.getDoubleTopic("ta").subscribe(0); //target area of image 0-100%
        ts = limelight.getDoubleTopic("ts").subscribe(0); //target skew/rotation of blue box degrees
        tl = limelight.getDoubleTopic("tl").subscribe(0); //pipeline latency ms
        tshort = limelight.getDoubleTopic("tshort").subscribe(0); //shortest fitted side length px
        tlong = limelight.getDoubleTopic("tlong").subscribe(0); //longest fitted side length px
        thor = limelight.getDoubleTopic("thor").subscribe(0); //width of rough box
        tvert = limelight.getDoubleTopic("tvert").subscribe(0); //height of rough box
        getpipe = limelight.getIntegerTopic("getpipe").subscribe(0); //active pipeline
        camtran = limelight.getDoubleArrayTopic("camtran").subscribe(new double[0]); //camera transform
        tid = limelight.getIntegerTopic("tid").subscribe(0); //primary apriltag id
        botpose = limelight.getDoubleArrayTopic("botpose").subscribe(new double[0]); //robot pose
        tcornxy = limelight.getDoubleArrayTopic("tcornxy").subscribe(new double[0]); //raw corner positions
        campose = limelight.getDoubleArrayTopic("campose").subscribe(new double[0]); //camera pose
        tc = limelight.getDoubleArrayTopic("tc").subscribe(new double[0]); //hsv under crosshair

        ledMode = limelight.getIntegerTopic("ledMode").publish();
        camMode = limelight.getIntegerTopic("camMode").publish();
        pipeline = limelight.getIntegerTopic("pipeline").publish();
        stream = limelight.getIntegerTopic("stream").publish();
        snapshot = limelight.getIntegerTopic("snapshot").publish();;
        crop = limelight.getDoubleArrayTopic("crop").publish();
    }

    public boolean hasTarget(){
        return tv.get() == 1;
    }

    public double getTargetX(){
        return tx.get();
    }

    public double getTargetY(){
        return ty.get();
    }

    public double getTargetArea(){
        return ta.get();
    }

    public double getTargetSkew(){
        return ts.get();
    }

    public double getFittedShortSideLength(){
        return tshort.get();
    }

    public double getFittedLongSideLength(){
        return tlong.get();
    }

    public double getRoughWidth(){
        return thor.get();
    }

    public double getRoughHeight(){
        return tvert.get();
    }

    public double getPipeline(){
        return getpipe.get();
    }

    public double[] getCameraTranslation(){
        return camtran.get();
    }

    public double getApriltagID(){
        return tid.get();
    }

    public double[] getCorners(){
        return tcornxy.get();
    }

    public double[] getCrosshairColor(){
        return tc.get();
    }

    public enum LEDMode{
        PIPELINE,
        OFF,
        BLINK,
        ON
    }

    public void setLedMode(LEDMode mode){
        ledMode.set(mode.ordinal());
    }

    public enum CamMode{
        VISION,
        DRIVER
    }

    public void setCamMode(CamMode mode){
        camMode.set(mode.ordinal());
    }

    public enum StreamMode{
        SIDE_BY_SIDE,
        PIP_MAIN,
        PIP_SECONDARY
    }

    public void setSecondaryCameraMode(StreamMode mode) {
        stream.set(mode.ordinal());
    }

    public void setCrop(double x1, double x2,  double y1, double y2){
        double[] cropArray = {x1, x2, y1, y2};
        crop.set(cropArray);
    }

    public void setPipeline(int pipeline){
        this.pipeline.set(pipeline);
    }

    /**
     * get the distance to an irregular object (e.g. cone or cube)
     * uses the closer approximation of both the width and the height.
     * @param objectHeight height of object (meters)
     * @param objectWidth width of object (meters)
     * @return approximate distance
     */
    public double getDistance(double objectHeight, double objectWidth, BoundingBox target){
        return (objectHeight * FOCAL_LENGTH) / getRoughHeight();
    }

    /**
     * get the equivalent of the yellow box in limelight software
     */
    public BoundingBox getBoundingBox(){
        var corns = tcornxy.get();
        double minX = 0, minY = 0, maxX = 0, maxY = 0;
        for(int i = 0; i < corns.length; i+=2){
            var x = corns[i];
            var y = corns[i+1];
            if(x < minX) minX = x;
            if(x > maxX) maxX = x;
            if(y < minY) minY = y;
            if(y > maxY) maxY = y;
        }
        return new BoundingBox(minX, minY, maxX, maxY);
    }


    /**
     * get limelight reported latency in seconds
     */
    public double getLatency(){
        return tl.get() / 1000;
    }

    double halfFieldWidth = 16.48/2;
    double halfFieldHeight = 8.1/2;

    /**
     * update a pose estimator from vision measurements
     * @param estimator the pose estimator to update
     */
    public void updatePoseEstimator(DifferentialDrivePoseEstimator estimator){
        TimestampedDoubleArray[] visionMeasurements = botpose.readQueue();
        for(TimestampedDoubleArray i : visionMeasurements){
            double time = Timer.getFPGATimestamp();// - getLatency();
            double[] val = i.value;
            if(val.length == 6) {
                estimator.addVisionMeasurement(new Pose2d(
                        val[0] + halfFieldWidth,
                        val[1] + halfFieldHeight,
                        Rotation2d.fromDegrees(val[5])
                )
                ,time);
            }
        }
    }

    /**
     * get the pose of the camera from apriltags
     */
    public Pose2d getCameraPose() {
        var camData = campose.get();
        if (camData.length == 3) {
            return new Pose2d(camData[0], camData[1], new Rotation2d());
        }
        return new Pose2d();
    }

    public static class BoundingBox{
        double x1, y1, x2, y2;
        public BoundingBox(double x1, double y1, double x2, double y2){
            this.x1 = x1;
            this.y1 = y1;
            this.x2 = x2;
            this.y2 = y2;
        }
        public Translation2d getCenter(){
            return new Translation2d((x1 + x2)/2, (y1 + y2) / 2);
        }
        public double getHeight(){
            return Math.abs(x2 - x1);
        }
        public double getWidth(){
            return Math.abs(y2 - y1);
        }
        public double getArea(){
            return getWidth()*getHeight();
        }
    }
}
