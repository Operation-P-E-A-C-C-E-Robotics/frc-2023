package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.DriveSignal;
import frc.lib.util.PeaccyDriveHelper;
import frc.lib.util.Util;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveTrain;

public class SeanyDrive extends CommandBase {
    private final Joystick driverJoystick;
    private final DriveTrain driveTrain;
    private final PeaccyDriveHelper driveHelper = new PeaccyDriveHelper();
    private boolean invertFront = false;
    private final RobotState robotState;

    // plan for fancyness:
    // - twist = turn in place quickly, but with huge deadband.
    // - button 1: shift to low gear
    // - button 2: fine control mode. (velocity control, small deadband, big curve, tiny sensitivity)
    // - button 3: fast acceleration mode. (velocity control, slew rate limited, auto shifting)
    // - button 4: cheesy quickturn
    // - button 5: enable gravity compensation (for charge station)
    // - button 6: lock drivetrain with position controller
    // - POV: switch drivetrain front, field relative
    // - button 7: reset odometry from apriltags
    // - button 8: zero odometry
    // - button 9: zero pigeon pitch + roll

    public SeanyDrive(Joystick driverJoystick, DriveTrain driveTrain, RobotState robotState){
        this.driverJoystick = driverJoystick;
        this.driveTrain = driveTrain;
        this.robotState = robotState;

        addRequirements(driveTrain);
    }

    @Override
    public void execute(){
        double throttle = driverJoystick.getY() * (invertFront ? 1 : -1);
        double wheel = driverJoystick.getX();
        // double arcadeTwist = Util.handleDeadbandWithSlopeIncrease(-driverJoystick.getTwist(), 0.5);
        var arcadeTwist = 0.0;
        boolean isHighGear = driverJoystick.getRawButton(1),
                isFineControl = driverJoystick.getRawButton(2),
                isFastAcceleration = driverJoystick.getRawButton(3),
                isQuickTurn = driverJoystick.getRawButton(4),
                isGravityCompensation = driverJoystick.getRawButton(5), //TODO
                isLockDrivetrain = driverJoystick.getRawButton(6), //TODO
                resetOdometryFromApriltags = driverJoystick.getRawButton(7),
                zeroOdometry = driverJoystick.getRawButton(8),
                zeroPigeon = driverJoystick.getRawButton(9);

        // double povAngle = driverJoystick.getPOV();
        // if((povAngle - robotState.getOdometryPose().getRotation().getDegrees()) > 90) invertFront = true;

        if (zeroOdometry) robotState.resetOdometry(new Pose2d());
        if (zeroPigeon) robotState.zeroImuPitchRoll();
        if (resetOdometryFromApriltags) robotState.resetOdometry(
                Util.toPose2d(robotState.getRawApriltagBotpose().get(new Pose3d()))
        );

        DriveSignal driveSignal;

        if(isFineControl) driveSignal = driveHelper.fineControl(throttle, wheel);
        else if (isFastAcceleration) driveSignal = driveHelper.fastAcceleration(
                throttle,
                wheel,
                driveTrain.getAverageVelocity(),
                driveTrain.isHighGear()
        );
        else driveSignal = driveHelper.curveDrive(
                throttle,
                wheel,
                arcadeTwist,
                isQuickTurn,
                isHighGear
        );

        driveTrain.set(driveSignal);
    }
}
