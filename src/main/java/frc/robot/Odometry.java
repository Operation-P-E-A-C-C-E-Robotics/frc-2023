package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.lib.sensors.Pigeon;
import frc.robot.Constants.DriveTrain;

class Odometry{
    private DriveTrain driveTrain;
    private Pigeon imu;
    private DifferentialDriveKinematics kinematics;
    private DifferentialDrivePoseEstimator estimator;
    private RobotContainer robot;

    public Odometry(RobotContainer robot) {
        this.robot = robot;
        imu = new Pigeon(new PigeonIMU(0)); //TODO PORT NO.
        kinematics = new DifferentialDriveKinematics(0); //TODO TRACK WIDTH
        estimator = new DifferentialDrivePoseEstimator(
            kinematics,
            imu.getRotation(),
            0, //TODO need drivetrain functions
            0,
            robot.getStartingPose(),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), //todo figure out whtf these are
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
        );
    }

    public void update() {
        if(hasNewVisionMeasurement()){
            estimator.addVisionMeasurement(getVisionPose(), getVisionTimestamp());
        }
        estimator.update(imu.getRotation(), 0,0); //todo need drivetrain functions
    }

    public Pose2d getPose() {
        return estimator.getEstimatedPosition();
    }

    public DifferentialDrivePoseEstimator getEstimator() {
        return estimator;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    //TODO placeholders
    private Pose2d getVisionPose(){
        return null;
    }
    private double getVisionTimestamp(){
        return 0;
    }
    private boolean hasNewVisionMeasurement() {
        return false;
    }
}
