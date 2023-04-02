package frc.robot;

import static frc.robot.Constants.DriveTrain.*;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.lib.sensors.PigeonHelper;
import frc.lib.util.PoseFilter;
import frc.lib.util.Util;
import frc.lib.safety.Value;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Supersystem;

public class RobotState {
    private final DriveTrain driveTrain;
    private final PigeonHelper imu;
    private final PoseFilter conePoseSmoothed = new PoseFilter(),
            cubePoseSmoothed = new PoseFilter(),
            visionTargetPoseSmoothed = new PoseFilter();
    private final DifferentialDrivePoseEstimator fieldToDrivetrainEstimator;
    private final Supersystem supersystem;
    private final Limelight drivetrainCameraMiddle, drivetrainCameraTwo;
    private final PowerDistribution pdp;
    private Pose3d prevConePose = new Pose3d();
    private Pose3d prevCubePose = new Pose3d();
    private Pose2d prevRobotPose, currentRobotPose;


    /**
     * A class to keep track of the position of different robot elements on the field.
     * @param driveTrain DriveTrain (for encoder values)
     * @param supersystem Supersystem (for supersystem position + kinematics)
     */
    public RobotState(DriveTrain driveTrain, Supersystem supersystem, PigeonHelper imu, Limelight apriltagCamera, Limelight armCamera){
        this.driveTrain = driveTrain;
        this.supersystem = supersystem;
        this.imu = imu;
        this.drivetrainCameraMiddle = apriltagCamera;
        this.drivetrainCameraTwo = armCamera;
        this.pdp = new PowerDistribution();
        fieldToDrivetrainEstimator = new DifferentialDrivePoseEstimator(
                DRIVE_KINEMATICS,
                imu.getRotation(),
                driveTrain.getLeftMeters(),
                driveTrain.getRightMeters(),
                new Pose2d(0, 0, new Rotation2d()), //TODO starting pose
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.01, 0.01, 0.005),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.5, 0.5, 3)
        );
        prevRobotPose = fieldToDrivetrainEstimator.getEstimatedPosition();
        currentRobotPose = prevRobotPose;
    }

     /**
     * update drivetrain odometry
     */
    public void update(){
        supersystem.getKinematics().reset();
        prevRobotPose = currentRobotPose;
        // apriltagCamera.updatePoseEstimatorOld(fieldToDrivetrainEstimator, driveTrain::getLeftMeters, driveTrain::getRightMeters, imu::getRotation);
        fieldToDrivetrainEstimator.update(imu.getRotation(), driveTrain.getLeftMeters(), driveTrain.getRightMeters());
        drivetrainCameraMiddle.updatePoseEstimator(fieldToDrivetrainEstimator, getOdometryPose(), driveTrain.getLeftVelocity(), driveTrain.getRightVelocity());
//        drivetrainCameraTwo.updatePoseEstimator(fieldToDrivetrainEstimator, getOdometryPose(), driveTrain.getLeftVelocity(), driveTrain.getRightVelocity());
        currentRobotPose = fieldToDrivetrainEstimator.getEstimatedPosition();

        DashboardManager.getInstance().drawDrivetrain(driveTrain.getDifferentialDrive(), getOdometryPose());
        var currentPoseOfEndEffector = supersystem.getKinematics().getEndEffectorPosition();
        var relativeToField = drivetrainToField(new Pose3d(currentPoseOfEndEffector.getMidPosition(), new Rotation3d()));
        DashboardManager.getInstance().drawEndEffector(new Pose2d(relativeToField.getX(), relativeToField.getY(), new Rotation2d()));

        var conePose = getConePose().get(new Pose3d());
        DashboardManager.getInstance().drawCone(new Pose2d(conePose.getX(), conePose.getY(), new Rotation2d()));
    }

    public void handleDrivetrainShift(){
        resetOdometry(prevRobotPose);
    }

    public static final double PLACE_DISTANCE = Units.inchesToMeters(43); //TODO meters
    public static final double PLACE_MAX_VELOCITY = 0.35; //TODO meters per second

    /**
     * determine whether the drivetrain is near enough a target to start placing or intaking
     * the drivetrain also needs to not be moving too fast.
     * @param target target to check
     * @return whether the drivetrain is near enough a target to start placing or intaking
     */
    public boolean inRangeOfTarget(Translation2d target){
        var pose = currentRobotPose.getTranslation();
        var distanceToTarget = pose.minus(target).getNorm();
        var distanceOkay = distanceToTarget < PLACE_DISTANCE;
        var velocityOkay = Math.abs(driveTrain.getAverageVelocity()) < PLACE_MAX_VELOCITY;
        return distanceOkay && velocityOkay;
    }

    public boolean isReadyToPlace(){
        var velocityOkay = Math.abs(driveTrain.getAverageVelocity()) < 0.1;
        return velocityOkay && drivetrainCameraMiddle.isOdometryCorrected();
    }

    public Value<Pose3d> getConePoseFromDrivetrainLimelight(){
        var poseRelativeToCamera = drivetrainCameraMiddle.getConePoseRelativeToCamera();
        if(!poseRelativeToCamera.isNormal()) return new Value<>(prevConePose);

        //note: before, x and y were negated to get them to be correct. does that still need to happen?
        var pose = drivetrainToField(apriltagCameraToDriveTrain(Util.toPose3d(poseRelativeToCamera.get(new Translation2d()))));

        if(drivetrainCameraMiddle.hasTarget()) pose = conePoseSmoothed.calculate(pose);
        prevConePose = pose;
        return new Value<>(pose);
    }

    public Value<Pose3d> getCubePoseFromDrivetrainLimelight(){
        var poseRelativeToCamera = drivetrainCameraMiddle.getCubePoseRelativeToCamera();
        if(!poseRelativeToCamera.isNormal()) return new Value<>(prevCubePose);

        //note: before, x and y were negated to get them to be correct. does that still need to happen?
        var pose = drivetrainToField(apriltagCameraToDriveTrain(Util.toPose3d(poseRelativeToCamera.get(new Translation2d()))));

        if(drivetrainCameraMiddle.hasTarget()) pose = cubePoseSmoothed.calculate(pose);
        prevCubePose = pose;
        return new Value<>(pose);
    }

    public Value<Pose3d> getCubePose(){
        var poseRelativeToCamera = drivetrainCameraTwo.getCubePoseRelativeToCamera();
        if(!poseRelativeToCamera.isNormal()) return getCubePoseFromDrivetrainLimelight();

        //note: before, x and y were negated to get them to be correct. does that still need to happen?
        var pose = drivetrainToField(endEffectorToDriveTrain(endEffectorCameraToEndEffector(Util.toPose3d(poseRelativeToCamera.get(new Translation2d())))));

        if(drivetrainCameraTwo.hasTarget()) pose = cubePoseSmoothed.calculate(pose);

        return new Value<>(pose);
    }

    public Value<Pose3d> getConePose(){
        var poseRelativeToCamera = drivetrainCameraTwo.getConePoseRelativeToCamera();
        // if(!poseRelativeToCamera.isNormal()) return getConePoseFromDrivetrainLimelight();

        //note: before, x and y were negated to get them to be correct. does that still need to happen?
        var pose = drivetrainToField(endEffectorToDriveTrain(endEffectorCameraToEndEffector(Util.toPose3d(poseRelativeToCamera.get(new Translation2d())))));

        // if(armCamera.hasTarget()) pose = conePoseSmoothed.calculate(pose);
        prevConePose = pose;
        return new Value<>(pose);
    }

    public Value<Pose3d> getVisionTargetPose(double z){
        var poseRelativeToCamera = drivetrainCameraTwo.getVisionTargetPoseRelativeToCamera();
        if(!poseRelativeToCamera.isNormal()) return Value.notAvailable();

        //note: before, x and y were negated to get them to be correct. does that still need to happen?
        var pose = drivetrainToField(turretToDrivetrain(endEffectorToTurret(endEffectorCameraToEndEffector(
                Util.toPose3d(poseRelativeToCamera.get(new Translation2d()), z)
        ))));

        if(drivetrainCameraTwo.hasTarget()) pose = visionTargetPoseSmoothed.calculate(pose);
        prevCubePose = pose;
        return new Value<>(pose);
    }

    public Value<Double> getConeAngleFromCamera(){
        drivetrainCameraTwo.setPipeline(Limelight.CONE_PIPELINE);
        if(drivetrainCameraTwo.getPipeline() != Limelight.CONE_PIPELINE) return Value.notAvailable();

        var angle = drivetrainCameraTwo.getTargetX();
        if(!angle.isNormal()) return Value.notAvailable();
        return angle;
    }

    public Value<Double> getCubeAngleFromCamera(){
        drivetrainCameraTwo.setPipeline(Limelight.CUBE_PIPELINE);
        if(drivetrainCameraTwo.getPipeline() != Limelight.CUBE_PIPELINE) return Value.notAvailable();

        var angle = drivetrainCameraTwo.getTargetX();
        if(!angle.isNormal()) return Value.notAvailable();
        return angle;
    }

    public boolean onRedAlliance(){
        return DriverStation.getAlliance() == DriverStation.Alliance.Red;
    }

    public boolean robotXInRange(double low, double high){
        return getOdometryPose().getTranslation().getX() > low && getOdometryPose().getTranslation().getX() < high;
    }

    public void zeroImuPitchRoll(){
        imu.zeroPitchRoll();
    }

    /**
     * get the robot's velocity in field space
     * @param dt time since last update
     * @return robot's velocity, relative to the field
     */
    public Transform3d getRobotVelocity(double dt){
        var deltaTransform = getOdometryPose().minus(prevRobotPose);
        return new Transform3d(
                new Translation3d(
                        deltaTransform.getX() / dt,
                        deltaTransform.getY() / dt,
                        0.0
                ), new Rotation3d(
                        0, 0,
                        deltaTransform.getRotation().getRadians() / dt
                )
        );
    }

    /**
     * get the robot's pose in field coordinates from the apriltag camera
     * @return robot's position, relative to the field
     */
    public Value<Pose3d> getRawApriltagBotpose(){
        return drivetrainCameraMiddle.getBotpose();
    }

    /**
     * get pose from odometry
     * @return robot's position, relative to the field
     */
    public Pose2d getOdometryPose(){
        return currentRobotPose;
    }

    /**
     * get the robot's pose in field coordinates as a Pose3d,
     * using fused odometry for x, y, and heading, and raw apriltag data for z
     * @return robot's position, relative to the field
     */
    public Pose3d getRobotPose(){
        Pose2d odometryPose = getOdometryPose();
        Pose3d botpose = drivetrainCameraMiddle.getBotpose().get(new Pose3d());
        return new Pose3d(
                odometryPose.getX(),
                odometryPose.getY(),
                botpose.getZ(),
                new Rotation3d(
                        imu.getRoll(),
                        imu.getPitch(),
                        odometryPose.getRotation().getRadians()
                )
        );
    }

    public PigeonHelper getPigeon(){
        return imu;
    }

    /**
     * reset drivetrain odometry
     * @param pose the robots current pose on the field
     */
    public void resetOdometry(Pose2d pose){
        fieldToDrivetrainEstimator.resetPosition(imu.getRotation(), driveTrain.getLeftMeters(), driveTrain.getRightMeters(), pose);
    }

    /**
     * convert a point on the field to it's robot-relative equivalent
     * @param fieldPoint any pose on the field
     * @return the pose relative to the center of the drivetrain
     */
    public Pose3d fieldToDrivetrain(Pose3d fieldPoint){
        Pose3d fieldToRobot = Util.toPose3d(getOdometryPose()); //todo try with snap to floor? (before used apriltag z)
        return Util.globalToLocalPose(fieldToRobot, fieldPoint);
    }

    /**
     * convert a drivetrain relative point to a field-relative point
     * @param drivetrainPoint a pose relative to the center of the drivetrain
     * @return the pose relative to the bottom left field corner
     */
    public Pose3d drivetrainToField(Pose3d drivetrainPoint){
        Pose3d fieldToRobot = Util.toPose3d(getOdometryPose());
        return Util.localToGlobalPose(fieldToRobot, drivetrainPoint);
    }

    /**
     * convert a pose relative to the center of the drivetrain
     * to be relative to the turret
     * @param drivetrainPoint a pose relative to the drivetrain
     * @return the same pose, rotated to account for turret position
     */
    public Pose3d drivetrainToTurret(Pose3d drivetrainPoint){
        var turretOrigin = new Pose3d(new Translation3d(),
                new Rotation3d(0,0,supersystem.getSupersystemState().getTurretAngle()));
        return Util.globalToLocalPose(turretOrigin, drivetrainPoint);
    }

    /**
     * convert a pose relative to the turret to be relative
     * to the drivetrain.
     * @param turretPoint a pose relative to the turret
     * @return the same pose rotated to account for turret position
     */
    public Pose3d turretToDrivetrain(Pose3d turretPoint){
        var turretOrigin = new Pose3d(new Translation3d(),
                new Rotation3d(0,0,supersystem.getSupersystemState().getTurretAngle()));
        return Util.localToGlobalPose(turretOrigin, turretPoint);
    }

    /**
     * convert a pose relative to the drivetrain to a pose
     * relative to the apriltag camera in the drivetrain
     * @param drivetrainPoint the pose relative to the center of the drivetrain
     * @return the same pose, offset to account for the camera position
     */
    public Pose3d drivetrainToApriltagCamera(Pose3d drivetrainPoint){
        var apriltagOrigin =  new Pose3d(0,0,0,
                new Rotation3d(0,0,Units.degreesToRadians(0))
        );
        return Util.globalToLocalPose(apriltagOrigin, drivetrainPoint);
    }

    /**
     * convert a pose relative to the apriltag camera
     * to a pose relative to the drivetrain.
     * @param apriltagCameraPoint the pose relative to the camera lens
     * @return the pose offset to account for the camera position
     */
    public Pose3d apriltagCameraToDriveTrain(Pose3d apriltagCameraPoint){
        var apriltagOrigin =  new Pose3d(0,0,0,
                new Rotation3d(0,0,Units.degreesToRadians(0))
        );
        return Util.localToGlobalPose(apriltagOrigin, apriltagCameraPoint);
    }

    /**
     * convert a pose relative to the turret to a
     * pose relative to the end effector.
     * @param turretPoint a pose relative to the center of the turret
     * @return the pose offset to account for the superstructure position.
     */
    public Pose3d turretToEndEffector(Pose3d turretPoint){
        Translation3d endEffectorPosition = supersystem.getKinematics()
                .getEndEffectorPosition()
                .getEndPosition();
        double endEffectorAngle = supersystem.getSupersystemState().getWristAngle() + Units.degreesToRadians(90); //TODO add 90?
        return Util.localToGlobalPose(new Pose3d(
                endEffectorPosition,
                new Rotation3d(0,endEffectorAngle,0)
        ), turretPoint);
    }

    /**
     * convert a pose relative to the end effector to
     * a pose relative to the turret.
     * @param endEffectorPoint a pose relative to the end effector
     * @return the pose offset to account for superstructure position
     */
    public Pose3d endEffectorToTurret(Pose3d endEffectorPoint){
        Translation3d endEffectorPosition = supersystem.getKinematics()
                .getEndEffectorPosition()
                .getEndPosition();
        double endEffectorAngle = supersystem.getSupersystemState().getWristAngle() + Units.degreesToRadians(90); //TODO add 90?
        return Util.globalToLocalPose(
                new Pose3d(endEffectorPosition, new Rotation3d(0,endEffectorAngle,0)),
                endEffectorPoint
        );
    }

    public Pose3d endEffectorToDriveTrain(Pose3d endEffectorPoint){
        Translation3d endEffectorPosition = supersystem.getKinematics()
                .getEndEffectorPosition()
                .getEndPosition();
        double wristPitch = supersystem.getSupersystemState().getWristAngle() + Units.degreesToRadians(90); //TODO add 90?
        double wristRoll = supersystem.getWristFlipAngle().getRadians();
        double turretYaw = supersystem.getSupersystemState().getTurretAngle();
        return Util.globalToLocalPose(
                new Pose3d(endEffectorPosition, new Rotation3d(wristRoll,wristPitch,turretYaw)),
                endEffectorPoint
        );
    }

    /**
     * convert from a pose relative to the end effector to
     * a pose relative to the center of a game piece held in the
     * end effector.
     * @param endEffectorPoint the pose relative to the end effector
     * @return the pose offset to account for where the game piece is held.
     */
    public Pose3d endEffectorToPlacePoint(Pose3d endEffectorPoint){
        var placePointOrigin = new Pose3d(
                -0.1,0,0, //TODO
                new Rotation3d()
        );
        return Util.globalToLocalPose(placePointOrigin, endEffectorPoint);
    }

    /**
     * convert from a pose relative to the center of a game piece
     * held in the end effector to the same pose relative to the
     * end effector
     * @param placePointPoint the pose relative to the place point
     * @return the pose offset to account for where the game piece is held
     */
    public Pose3d placePointToEndEffector(Pose3d placePointPoint){
        var placePointOrigin = new Pose3d(
                -0.1,0,0, //TODO
                new Rotation3d()
        );
        return Util.localToGlobalPose(placePointOrigin, placePointPoint); //TODO offset and check math
    }

    /**
     * convert from a pose relative to the end effector
     * to a pose relative to the camera on the end effector
     * @param endEffectorPoint pose relative to the end effector
     * @return pose offset to account for camera position
     */
    public Pose3d endEffectorToEndEffectorCamera(Pose3d endEffectorPoint){
        var endEffectorCameraOrigin = new Pose3d();
        return Util.globalToLocalPose(endEffectorCameraOrigin, endEffectorPoint); //todo globalToLocal or localToGlobal?
    }

    /**
     * convert from a pose relative to the end effector camera to a pose
     * relative to the end effector
     * @param endEffectorCameraPoint pose relative to the end effector
     * @return pose with offset to account for camera position
     */
    public Pose3d endEffectorCameraToEndEffector(Pose3d endEffectorCameraPoint){
        var endEffectorCameraOrigin = new Pose3d();
        return Util.localToGlobalPose(endEffectorCameraOrigin, endEffectorCameraPoint); //todo globalToLocal or localToGlobal?
    }
}
