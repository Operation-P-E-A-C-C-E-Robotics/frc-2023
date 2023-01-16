package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.sensors.Pigeon;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.Auto.*;

public class Odometry{
    private final DriveTrain driveTrain;
    private final Pigeon imu;
    private final DifferentialDriveKinematics kinematics;
    private final DifferentialDrivePoseEstimator estimator;

    public Odometry(RobotContainer robot, DriveTrain driveTrain) {
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

    public Rotation2d getHeading(){
        return imu.getRotation();
    }

    public double getYaw(){
        return imu.getYaw();
    }

    public double getPitch(){
        SmartDashboard.putNumber("pitch", imu.getPitch());
        return imu.getPitch();
    }

    public double getRoll(){
        return imu.getRoll();
    }
}