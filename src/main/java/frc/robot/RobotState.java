package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.sensors.Pigeon;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Supersystem;

import static frc.robot.Constants.Auto.PIGEON_IMU;
import static frc.robot.Constants.Auto.TRACK_WIDTH;

public class RobotState {
    private final DriveTrain driveTrain;
    private final Pigeon imu;

    private final DifferentialDriveKinematics driveKinematics;
    private final DifferentialDrivePoseEstimator fieldToDrivetrainEstimator;
    private final Supersystem supersystem;

    public enum Frame{
        FIELD, //origin bottom left of field with alliance on left
        DRIVETRAIN, //origin center of drivetrain, towards front of robot
        TURRET, //origin center of turret, towards front of turret
        END_EFFECTOR, //origin tip of end effector, towards front of turret.
        PLACE_POINT, //origin middle of end effector, towards front of turret.
        APRILTAG_CAMERA, //origin camera lens, towards camera front
        END_EFFECTOR_CAMERA, //origin camera lens, towards camera front
    }

    public RobotState(RobotContainer robot, DriveTrain driveTrain, Supersystem supersystem){
        this.driveTrain = driveTrain;
        this.supersystem = supersystem;
        imu = new Pigeon(new PigeonIMU(PIGEON_IMU));
        driveKinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
        fieldToDrivetrainEstimator = new DifferentialDrivePoseEstimator(
                driveKinematics,
                imu.getRotation(),
                driveTrain.getLeftEncoder(),
                driveTrain.getRightEncoder(),
                robot.getStartingPose(),
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), //todo figure out wtf these are
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
        );
    }

    public void update(){
        fieldToDrivetrainEstimator.update(imu.getRotation(), driveTrain.getLeftEncoder(),driveTrain.getRightEncoder());
    }

    public Pose2d getOdometryPose(){
        return fieldToDrivetrainEstimator.getEstimatedPosition();
    }
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

    public Pose3d drivetrainToTurret(Pose3d drivetrainPoint){
        return drivetrainPoint.relativeTo(new Pose3d(
                0,0,0, //turret center in robot
                new Rotation3d(0,0,supersystem.getSupersystemState().getTurretAngle())
        ));
    }

    public Pose3d turretToDrivetrain(Pose3d turretPoint){
        return turretPoint.plus(new Transform3d(
                new Translation3d(0,0,0), //turret center in robot
                new Rotation3d(0,0,supersystem.getSupersystemState().getTurretAngle())
        ));
    }

    public Pose3d drivetrainToApriltagCamera(Pose3d drivetrainPoint){
        return drivetrainPoint.relativeTo(new Pose3d(
                0,0,0, //TODO camera center in robot
                new Rotation3d(0,0,0) //TODO camera rotation in robot
        ));
    }

    public Pose3d apriltagCameraToDriveTrain(Pose3d apriltagCameraPoint){
        return apriltagCameraPoint.plus(new Transform3d(
                new Translation3d(0,0,0), //TODO camera center in robot
                new Rotation3d(0,0,0) //TODO camera rotation in robot
        ));
    }

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

    public Pose3d endEffectorToTurret(Pose3d endEffectorPoint){
        Translation3d endEffectorPosition = supersystem.getKinematics()
                .getEndEffectorPosition()
                .getEndPosition();
        double endEffectorAngle = supersystem.getSupersystemState().getWristAngle() + Units.degreesToRadians(90); //TODO add 90?
        return endEffectorPoint.plus(new Transform3d(endEffectorPosition, new Rotation3d(0, endEffectorAngle, 0)));
    }

    public Pose3d endEffectorToPlacePoint(Pose3d endEffectorPoint){
        return endEffectorPoint.relativeTo(new Pose3d(
                -0.1,0,0, //TODO offset
                new Rotation3d()
        ));
    }

    public Pose3d placePointToEndEffector(Pose3d placePointPoint){
        return placePointPoint.plus(new Transform3d(
                new Translation3d(-0.1,0,0),
                new Rotation3d()
        )); //TODO offset and check math
    }

    public Pose3d endEffectorToEndEffectorCamera(Pose3d endEffectorPoint){
        return endEffectorPoint.relativeTo(new Pose3d(
                0,0,0, //TODO camera position
                new Rotation3d(0,0,0)
        ));
    }

    public Pose3d endEffectorCameraToEndEffector(Pose3d endEffectorPoint){
        return endEffectorPoint.plus(new Transform3d(
                new Translation3d(0,0,0), //TODO camera position
                new Rotation3d(0,0,0)
        ));
    }
}
