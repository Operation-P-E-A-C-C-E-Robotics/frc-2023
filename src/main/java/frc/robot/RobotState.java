package frc.robot;

import static frc.robot.Constants.DriveTrain.*;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.sensors.PigeonHelper;
import frc.lib.util.AveragePose;
import frc.lib.util.Util;
import frc.lib.safety.Value;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Supersystem;

public class RobotState {
    private final DriveTrain driveTrain;
    private final PigeonHelper imu;
    private final AveragePose conePoseSmoothed = new AveragePose(),
            cubePoseSmoothed = new AveragePose(),
            visionTargetPoseSmoothed = new AveragePose();
    private final DifferentialDrivePoseEstimator fieldToDrivetrainEstimator;
    private final Supersystem supersystem;
    private final Limelight apriltagCamera, armCamera;
    private Pose3d prevConePose = new Pose3d();
    private Pose3d prevCubePose = new Pose3d();
    private Pose2d prevRobotPose;


    /**
     * A class to keep track of the position of different robot elements on the field.
     * @param driveTrain DriveTrain (for encoder values)
     * @param supersystem Supersystem (for supersystem position + kinematics)
     */
    public RobotState(DriveTrain driveTrain, Supersystem supersystem, PigeonHelper imu, Limelight apriltagCamera, Limelight armCamera){
        this.driveTrain = driveTrain;
        this.supersystem = supersystem;
        this.imu = imu;
        this.apriltagCamera = apriltagCamera;
        this.armCamera = armCamera;
        fieldToDrivetrainEstimator = new DifferentialDrivePoseEstimator(
                DRIVE_KINEMATICS,
                imu.getRotation(),
                driveTrain.getLeftMeters(),
                driveTrain.getRightMeters(),
                new Pose2d(3, 3, new Rotation2d()), //TODO starting pose
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(1, 1, 0.5)
        );
        prevRobotPose = fieldToDrivetrainEstimator.getEstimatedPosition();
    }

    public static final double PLACE_DISTANCE = 2; //TODO meters
    public static final double PLACE_MAX_VELOCITY = 1; //TODO meters per second

    /**
     * determine whether the drivetrain is near enough a target to start placing or intaking
     * the drivetrain also needs to not be moving too fast.
     * @param target target to check
     * @return whether the drivetrain is near enough a target to start placing or intaking
     */
    public boolean inRangeOfTarget(Translation2d target){
        var pose = getOdometryPose().getTranslation();
        var distanceToTarget = pose.minus(target).getNorm();
        var distanceOkay = distanceToTarget < PLACE_DISTANCE;
        var velocityOkay = Math.abs(driveTrain.getAverageVelocity()) < PLACE_MAX_VELOCITY;
        return distanceOkay && velocityOkay;
    }

    public Value<Pose3d> getConePoseFromDrivetrainLimelight(){
        var poseRelativeToCamera = apriltagCamera.getConePoseRelativeToCamera();
        if(!poseRelativeToCamera.isNormal()) return new Value<>(prevConePose);

        //note: before, x and y were negated to get them to be correct. does that still need to happen?
        var pose = drivetrainToField(apriltagCameraToDriveTrain(Util.toPose3d(poseRelativeToCamera.get(new Translation2d()))));

        if(apriltagCamera.hasTarget()) pose = conePoseSmoothed.calculate(pose);
        prevConePose = pose;
        return new Value<>(pose);
    }

    public Value<Pose3d> getCubePoseFromDrivetrainLimelight(){
        var poseRelativeToCamera = apriltagCamera.getCubePoseRelativeToCamera();
        if(!poseRelativeToCamera.isNormal()) return new Value<>(prevCubePose);

        //note: before, x and y were negated to get them to be correct. does that still need to happen?
        var pose = drivetrainToField(apriltagCameraToDriveTrain(Util.toPose3d(poseRelativeToCamera.get(new Translation2d()))));

        if(apriltagCamera.hasTarget()) pose = cubePoseSmoothed.calculate(pose);
        prevCubePose = pose;
        return new Value<>(pose);
    }

    public Value<Pose3d> getCubePose(){
        var poseRelativeToCamera = armCamera.getCubePoseRelativeToCamera();
        if(!poseRelativeToCamera.isNormal()) return getCubePoseFromDrivetrainLimelight();

        //note: before, x and y were negated to get them to be correct. does that still need to happen?
        var pose = drivetrainToField(turretToDrivetrain(endEffectorToTurret(endEffectorCameraToEndEffector(Util.toPose3d(poseRelativeToCamera.get(new Translation2d()))))));

        if(armCamera.hasTarget()) pose = cubePoseSmoothed.calculate(pose);

        return new Value<>(pose);
    }

    public Value<Pose3d> getConePose(){
        var poseRelativeToCamera = armCamera.getConePoseRelativeToCamera();
        if(!poseRelativeToCamera.isNormal()) return getConePoseFromDrivetrainLimelight();

        //note: before, x and y were negated to get them to be correct. does that still need to happen?
        var pose = drivetrainToField(turretToDrivetrain(endEffectorToTurret(endEffectorCameraToEndEffector(Util.toPose3d(poseRelativeToCamera.get(new Translation2d()))))));

        if(armCamera.hasTarget()) pose = conePoseSmoothed.calculate(pose);
        prevConePose = pose;
        return new Value<>(pose);
    }

    public Value<Pose3d> getVisionTargetPose(double z){
        var poseRelativeToCamera = armCamera.getVisionTargetPoseRelativeToCamera();
        if(!poseRelativeToCamera.isNormal()) return Value.notAvailable();

        //note: before, x and y were negated to get them to be correct. does that still need to happen?
        var pose = drivetrainToField(turretToDrivetrain(endEffectorToTurret(endEffectorCameraToEndEffector(
                Util.toPose3d(poseRelativeToCamera.get(new Translation2d()), z)
        ))));

        if(armCamera.hasTarget()) pose = visionTargetPoseSmoothed.calculate(pose);
        prevCubePose = pose;
        return new Value<>(pose);
    }

    public Value<Double> getConeAngleFromCamera(){
        armCamera.setPipeline(Limelight.CONE_PIPELINE);
        if(armCamera.getPipeline() != Limelight.CONE_PIPELINE) return Value.notAvailable();

        var angle = armCamera.getTargetX();
        if(!angle.isNormal()) return Value.notAvailable();
        return angle;
    }

    public Value<Double> getCubeAngleFromCamera(){
        armCamera.setPipeline(Limelight.CUBE_PIPELINE);
        if(armCamera.getPipeline() != Limelight.CUBE_PIPELINE) return Value.notAvailable();

        var angle = armCamera.getTargetX();
        if(!angle.isNormal()) return Value.notAvailable();
        return angle;
    }

    /**
     * update drivetrain odometry
     */
    public void update(){
        supersystem.getKinematics().reset(); //TODO better place for this?
        prevRobotPose = fieldToDrivetrainEstimator.getEstimatedPosition();
        apriltagCamera.updatePoseEstimator(fieldToDrivetrainEstimator, driveTrain::getLeftMeters, driveTrain::getRightMeters, imu::getRotation);
        fieldToDrivetrainEstimator.updateWithTime(Timer.getFPGATimestamp(), imu.getRotation(), driveTrain.getLeftMeters(),driveTrain.getRightMeters());

        if (Robot.isReal()) {
            DashboardManager.getInstance().drawDrivetrain(driveTrain.getDifferentialDrive(), getOdometryPose());
        } else {
            DashboardManager.getInstance().drawDrivetrain(driveTrain.getDifferentialDrive(), getOdometryPose());
            var currentPoseOfEndEffector = supersystem.getKinematics().getEndEffectorPosition();
            var relativeToField = drivetrainToField(new Pose3d(currentPoseOfEndEffector.getMidPosition(), new Rotation3d()));
            DashboardManager.getInstance().drawEndEffector(new Pose2d(relativeToField.getX(), relativeToField.getY(), new Rotation2d()));
        }

        var conePose = getConePoseFromDrivetrainLimelight().get(new Pose3d());
        DashboardManager.getInstance().drawCone(new Pose2d(conePose.getX(), conePose.getY(), new Rotation2d()));
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
        return apriltagCamera.getBotpose();
    }

    /**
     * get pose from odometry
     * @return robot's position, relative to the field
     */
    public Pose2d getOdometryPose(){
        return fieldToDrivetrainEstimator.getEstimatedPosition();
    }

    /**
     * get the robot's pose in field coordinates as a pose3d,
     * using fused odometry for x, y, and heading, and raw apriltag data for z
     * @return robot's position, relative to the field
     */
    public Pose3d getRobotPose(){
        Pose2d odometryPose = getOdometryPose();
        Pose3d botpose = apriltagCamera.getBotpose().get(new Pose3d());
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
        Pose3d fieldToRobot = getRobotPose();
        return Util.globalToLocalPose(fieldToRobot, fieldPoint);
    }

    /**
     * convert a drivetrain relative point to a field-relative point
     * @param drivetrainPoint a pose relative to the center of the drivetrain
     * @return the pose relative to the bottom left field corner
     */
    public Pose3d drivetrainToField(Pose3d drivetrainPoint){
        Pose3d fieldToRobot = getRobotPose();
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
                new Rotation3d(0,0,Units.degreesToRadians(180)) //todo fix 180
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
                new Rotation3d(0,0,Units.degreesToRadians(180))
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
        return Util.globalToLocalPose(endEffectorCameraOrigin, endEffectorPoint); //todo globaltolocal or localtoglobal?
    }

    /**
     * convert from a pose relative to the end effector camera to a pose
     * relative to the end effector
     * @param endEffectorCameraPoint pose relative to the end effector
     * @return pose with offset to account for camera position
     */
    public Pose3d endEffectorCameraToEndEffector(Pose3d endEffectorCameraPoint){
        var endEffectorCameraOrigin = new Pose3d();
        return Util.localToGlobalPose(endEffectorCameraOrigin, endEffectorCameraPoint); //todo globaltolocal or localtoglobal?
    }
}
