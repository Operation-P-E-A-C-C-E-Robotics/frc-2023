package frc.robot.commands.auto;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.field.ChargeStation;
import frc.lib.motion.TrapezoidalMotion;
import frc.lib.sensors.PigeonHelper;
import frc.lib.util.DriveSignal;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class ModelBalancer extends CommandBase{
    private static final ChargeStation model = new ChargeStation();
    private static final double DEADBAND = 0.1; //radians from center
    private static final double BANGBANGSPEED = 0.3; //drivetrain percentage
    private static final TrapezoidalMotion driveMotion = new TrapezoidalMotion(Constants.DriveTrain.AUTO_MAX_SPEED_METERS_PER_SECOND, Constants.DriveTrain.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    private final DriveTrain driveTrain;
    private final PigeonHelper imu;
    private double prevPitch = 0, prevVelocity = 0;
    private final double goalPosition = 0;

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
        double pitchAcceleration = (pitchVelocity - prevVelocity)/0.02;
        prevPitch = currentPitch;
        prevVelocity = pitchVelocity;

        if(Math.abs(pitchVelocity) < 0.1){
            //model won't work if we aren't accelerating
            bangBangController(currentPitch, driveTrain);
        }

        double currentPositionOnModel = model.calculateRobotPosition(currentPitch, pitchAcceleration);

        driveMotion.setCurrentState(driveTrain.getAverageEncoderDistance(), driveTrain.getAverageVelocity());
        driveMotion.setGoalState(currentPositionOnModel, 0);
        var velocity = driveMotion.calculate(0.02).velocity;
        DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(
                velocity,
                velocity
        );
        driveTrain.set(DriveSignal.velocityDrive(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond, true));
    }
    static double maxBangBangSpeed = 0.5, minBangBangSpeed = 0.2;

    static void bangBangController(double currentPitch, DriveTrain driveTrain) {
        double left, right;
        double error = Math.abs(currentPitch);
        double speed = Util.interpolate(minBangBangSpeed, maxBangBangSpeed, error/20);
        if (currentPitch > DEADBAND) {
            left = -speed;
            right = -speed;
        } else if (currentPitch < -DEADBAND) {
            left = speed;
            right = speed;
        } else {
            left = 0;
            right = 0;
        }
        driveTrain.set(DriveSignal.velocityDrive(left, right, true));
    }
}
