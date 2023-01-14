package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.sensors.Pigeon;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.Auto.*;

public class Odometry{
    private DriveTrain driveTrain;
    private Pigeon imu;
    private DifferentialDriveKinematics kinematics;
    private DifferentialDrivePoseEstimator estimator;
    private RobotContainer robot;

    public Odometry(RobotContainer robot, DriveTrain driveTrain) {
        this.robot = robot;
        this.driveTrain = driveTrain;
        imu = new Pigeon(new PigeonIMU(PIGEON_IMU));
        kinematics = new DifferentialDriveKinematics(TRACK_WIDTH);
        estimator = new DifferentialDrivePoseEstimator(
            kinematics,
            imu.getRotation(),
            driveTrain.getLeftEncoder(),
            driveTrain.getRightEncoder(),
            robot.getStartingPose(),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), //todo figure out wtf these are
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)
        );
    }

    public void update() {
        if(hasNewVisionMeasurement()){
            estimator.addVisionMeasurement(getVisionPose(), getVisionTimestamp());
        }
        estimator.update(imu.getRotation(), driveTrain.getLeftEncoder(),driveTrain.getRightEncoder());
        SmartDashboard.putNumber("x", getPose().getX());
        SmartDashboard.putNumber("y", getPose().getY());
        SmartDashboard.putNumber("rot", getPose().getRotation().getDegrees());
    }

    public void resetPose(Pose2d pose){
        estimator.resetPosition(imu.getRotation(), driveTrain.getLeftEncoder(), driveTrain.getRightEncoder(), pose);
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
