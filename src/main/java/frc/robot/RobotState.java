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
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Supersystem;

public class RobotState {
    private final DriveTrain driveTrain;
    private final PigeonHelper imu;
    private final AveragePose conePoseSmoothed = new AveragePose();
    private final DifferentialDrivePoseEstimator fieldToDrivetrainEstimator;
    private final Supersystem supersystem;
    private Limelight apriltagCamera, armCamera;
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
                new Pose2d(10, 5, new Rotation2d()), //TODO starting pose
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.2, 0.2, 0.02)
        );
        prevRobotPose = fieldToDrivetrainEstimator.getEstimatedPosition();
    }

    public Pose3d getConePose(){
        var poseRelativeToCamera = apriltagCamera.getConePoseRelativeToCamera();
        //note: before, x and y were negated to get them to be correct. does that still need to happen?
        var pose = drivetrainToField(apriltagCameraToDriveTrain(Util.toPose3d(poseRelativeToCamera)));

        if(apriltagCamera.hasTarget()) pose = conePoseSmoothed.calculate(pose);

        return pose;
    }

    /**
     * update drivetrain odometry
     */
    public void update(){
        prevRobotPose = fieldToDrivetrainEstimator.getEstimatedPosition();
        apriltagCamera.updatePoseEstimator(fieldToDrivetrainEstimator);
        fieldToDrivetrainEstimator.updateWithTime(Timer.getFPGATimestamp(), imu.getRotation(), driveTrain.getLeftMeters(),driveTrain.getRightMeters());
        if (Robot.isReal()) {
            DashboardManager.getInstance().drawDrivetrain(driveTrain.getDifferentialDrive(), getOdometryPose());
        } else {
            // driveTrain.driveSim.setPose(getOdometryPose());
            DashboardManager.getInstance().drawDrivetrain(driveTrain.getDifferentialDrive(), driveTrain.driveSim.getPose());
        }
        DashboardManager.getInstance().drawAprilTag(apriltagCamera.getCameraPose());
        var conePose = getConePose();
        DashboardManager.getInstance().drawCone(new Pose2d(conePose.getX(), conePose.getY(), new Rotation2d()));
    }

    public Transform3d getRobotVelocity(){
        var t2d = getOdometryPose().minus(prevRobotPose);
        return new Transform3d(new Translation3d(t2d.getX(), t2d.getY(), 0.0), new Rotation3d(0,0,t2d.getRotation().getRadians()));
    }

    /**
     * get pose from odometry
     * @return robot's position, relative to the field
     */
    public Pose2d getOdometryPose(){
        return fieldToDrivetrainEstimator.getEstimatedPosition();
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
        Pose2d fieldToRobot = fieldToDrivetrainEstimator.getEstimatedPosition();
        return fieldPoint.relativeTo(new Pose3d(
                fieldToRobot.getX(),
                fieldToRobot.getY(),
                0, //TODO apriltag z
                new Rotation3d(
                        Units.degreesToRadians(imu.getRoll()),
                        Units.degreesToRadians(imu.getPitch()),
                        imu.getRotation().getRadians()
                )
        ));
    }

    // public Pose3d fieldOriginFromDrivetrain(){
        
    // }

    /**
     * convert a drivetrain relative point to a field-relative point
     * @param drivetrainPoint a pose relative to the center of the drivetrain
     * @return the pose relative to the bottom left field corner
     */
    public Pose3d drivetrainToField(Pose3d drivetrainPoint){
        Pose2d fieldToRobot = fieldToDrivetrainEstimator.getEstimatedPosition();
        Pose3d fieldToRobot3d = Util.toPose3d(fieldToRobot,0,0,0);
        return Util.localToGlobalPose(fieldToRobot3d, drivetrainPoint);
    }

    /**
     * convert a pose relative to the center of the drivetrain
     * to be relative to the turret
     * @param drivetrainPoint a pose relative to the drivetrain
     * @return the same pose, rotated to account for turret position
     */
    public Pose3d drivetrainToTurret(Pose3d drivetrainPoint){
        return drivetrainPoint.relativeTo(new Pose3d(
                0,0,0, //turret center in robot
                new Rotation3d(0,0,supersystem.getSupersystemState().getTurretAngle())
        ));
    }

    /**
     * convert a pose relative to the turret to be relative
     * to the drivetrain.
     * @param turretPoint a pose relative to the turret
     * @return the same pose rotated to account for turret position
     */
    public Pose3d turretToDrivetrain(Pose3d turretPoint){
        return turretPoint.plus(new Transform3d(
                new Translation3d(0,0,0), //turret center in robot
                new Rotation3d(0,0,supersystem.getSupersystemState().getTurretAngle())
        ));
    }

    /**
     * convert a pose relative to the drivetrain to a pose
     * relative to the apriltag camera in the drivetrain
     * @param drivetrainPoint the pose relative to the center of the drivetrain
     * @return the same pose, offset to account for the camera position
     */
    public Pose3d drivetrainToApriltagCamera(Pose3d drivetrainPoint){
        return drivetrainPoint.relativeTo(new Pose3d(
                0,0,0, //TODO camera center in robot
                new Rotation3d(0,0,0) //TODO camera rotation in robot
        ));
    }

    /**
     * convert a pose relative to the apriltag camera
     * to a pose relative to the drivetrain.
     * @param apriltagCameraPoint the pose relative to the camera lens
     * @return the pose offset to account for the camera position
     */
    public Pose3d apriltagCameraToDriveTrain(Pose3d apriltagCameraPoint){
        return apriltagCameraPoint.relativeTo(new Pose3d(
                new Translation3d(0,0,0), //TODO camera center in robot
                new Rotation3d(0,0,Units.degreesToRadians(180)) //TODO camera rotation in robot
        ));
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
        return turretPoint.relativeTo(new Pose3d(
                endEffectorPosition,
                new Rotation3d(0,endEffectorAngle,0)
        ));
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
        return endEffectorPoint.plus(new Transform3d(endEffectorPosition, new Rotation3d(0, endEffectorAngle, 0)));
    }

    /**
     * convert from a pose relative to the end effector to
     * a pose relative to the center of a game piece held in the
     * end effector.
     * @param endEffectorPoint the pose relative to the end effector
     * @return the pose offset to account for where the game piece is held.
     */
    public Pose3d endEffectorToPlacePoint(Pose3d endEffectorPoint){
        return endEffectorPoint.relativeTo(new Pose3d(
                -0.1,0,0, //TODO offset
                new Rotation3d()
        ));
    }

    /**
     * convert from a pose relative to the center of a game piece
     * held in the end effector to the same pose relative to the
     * end effector
     * @param placePointPoint the pose relative to the place point
     * @return the pose offset to account for where the game piece is held
     */
    public Pose3d placePointToEndEffector(Pose3d placePointPoint){
        return placePointPoint.plus(new Transform3d(
                new Translation3d(-0.1,0,0),
                new Rotation3d()
        )); //TODO offset and check math
    }

    /**
     * convert from a pose relative to the end effector
     * to a pose relative to the camera on the end effector
     * @param endEffectorPoint pose relative to the end effector
     * @return pose offset to account for camera position
     */
    public Pose3d endEffectorToEndEffectorCamera(Pose3d endEffectorPoint){
        return endEffectorPoint.relativeTo(new Pose3d(
                0,0,0, //TODO camera position
                new Rotation3d(0,0,0)
        ));
    }

    /**
     * convert from a pose relative to the end effector camera to a pose
     * relative to the end effector
     * @param endEffectorPoint pose relative to the end effector
     * @return pose with offset to account for camera position
     */
    public Pose3d endEffectorCameraToEndEffector(Pose3d endEffectorPoint){
        return endEffectorPoint.plus(new Transform3d(
                new Translation3d(0,0,0), //TODO camera position
                new Rotation3d(0,0,0)
        ));
    }
}
