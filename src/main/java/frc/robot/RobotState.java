package frc.robot;

import static frc.robot.Constants.DriveTrain.PIGEON_IMU;
import static frc.robot.Constants.DriveTrain.TRACK_WIDTH;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.sensors.Pigeon;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Supersystem;

public class RobotState {
    private final DriveTrain driveTrain;
    private final Pigeon imu;

    private final DifferentialDriveKinematics driveKinematics; //consider moving to constants
    private final DifferentialDrivePoseEstimator fieldToDrivetrainEstimator;
    private final Supersystem supersystem;



    /**
     * A class to keep track of the position of different robot elements on the field.
     * @param robot RobotContainer (for initial pose)
     * @param driveTrain DriveTrain (for encoder values)
     * @param supersystem Supersystem (for supersystem position + kinematics)
     */
    public RobotState(DriveTrain driveTrain, Supersystem supersystem){
        this.driveTrain = driveTrain;
        this.supersystem = supersystem;
        imu = new Pigeon(new PigeonIMU(PIGEON_IMU));
        driveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
        fieldToDrivetrainEstimator = new DifferentialDrivePoseEstimator(
                driveKinematics,
                imu.getRotation(),
                driveTrain.getLeftMeters(),
                driveTrain.getRightMeters(),
                new Pose2d(), //TODO starting pose
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), //TODO figure out wtf these are
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
        );
    }

    /**
     * update drivetrain odometry
     */
    public void update(){
        fieldToDrivetrainEstimator.update(imu.getRotation(), driveTrain.getLeftMeters(),driveTrain.getRightMeters());
        
       
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
     * @return the drivetrain's kinematics
     */
    public DifferentialDriveKinematics getDriveKinematics(){
        return driveKinematics;
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

    /**
     * convert a drivetrain relative point to a field-relative point
     * @param drivetrainPoint a pose relative to the center of the drivetrain
     * @return the pose relative to the bottom left field corner
     */
    public Pose3d drivetrainToField(Pose3d drivetrainPoint){
        Pose2d fieldToRobot = fieldToDrivetrainEstimator.getEstimatedPosition();
        return new Pose3d(
                fieldToRobot.getX(),
                fieldToRobot.getY(),
                0, //TODO apriltag z
                new Rotation3d(
                        Units.degreesToRadians(imu.getRoll()), //TODO better yaw pitch roll
                        Units.degreesToRadians(imu.getPitch()),
                        imu.getRotation().getRadians()
                )
        ).plus(new Transform3d(
                drivetrainPoint.getTranslation(),
                drivetrainPoint.getRotation()
        ));
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
        return apriltagCameraPoint.plus(new Transform3d(
                new Translation3d(0,0,0), //TODO camera center in robot
                new Rotation3d(0,0,0) //TODO camera rotation in robot
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
