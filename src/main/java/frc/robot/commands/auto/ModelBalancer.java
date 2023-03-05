package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.field.ChargeStation;
import frc.lib.motion.TrapezoidalMotion;
import frc.lib.sensors.PigeonHelper;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class ModelBalancer extends CommandBase{
    private static final ChargeStation model = new ChargeStation();
    private static final TrapezoidalMotion driveMotion = new TrapezoidalMotion(Constants.DriveTrain.AUTO_MAX_SPEED_METERS_PER_SECOND, Constants.DriveTrain.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    private final DriveTrain driveTrain;
    private final PigeonHelper imu;
    private double prevPitch = 0;

    public ModelBalancer(DriveTrain driveTrain, PigeonHelper imu) {
        this.driveTrain = driveTrain;
        this.imu = imu;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize(){
        prevPitch = imu.getPitch();
    }

    @Override
    public void execute(){
        double currentPitch = imu.getPitch();
        double pitchVelocity = (currentPitch - prevPitch)/0.02;
        prevPitch = currentPitch;

        if(Math.abs(pitchVelocity) < 0.1){
            //TODO bad data if we're not moving fast enough.
            driveTrain.tankDrive(0,0);
        }

        double currentPositionOnModel = model.calculateRobotPosition(currentPitch, pitchVelocity);
        driveMotion.setCurrentState(driveTrain.getAverageEncoderDistance(), driveTrain.getAverageVelocity());
        driveMotion.setGoalState(currentPositionOnModel, 0);
        DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(
                driveMotion.calculate(0.02).velocity,
                driveMotion.calculate(0.02).velocity
        );
        driveTrain.velocityDriveLQR(speeds);
    }
}
