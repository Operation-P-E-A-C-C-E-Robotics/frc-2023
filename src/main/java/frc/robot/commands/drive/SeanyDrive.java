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
        double arcadeTwist = Util.handleDeadbandWithSlopeIncrease(-driverJoystick.getTwist(), 0.5);
        boolean isHighGear = !driverJoystick.getRawButton(3),
                isFineControl = driverJoystick.getRawButton(7),
                isFastAcceleration = driverJoystick.getRawButton(6),
                isQuickTurn = driverJoystick.getRawButton(1),
                isGravityCompensation = driverJoystick.getRawButton(5), //TODO
                isLockDrivetrain = driverJoystick.getRawButton(6), //TODO
                resetOdometryFromApriltags = driverJoystick.getRawButton(13),
                zeroOdometry = driverJoystick.getRawButton(12),
                zeroPigeon = driverJoystick.getRawButton(11);

        //use the pov hat to determine the front of the robot.
         double povAngle = driverJoystick.getPOV();
         if(povAngle != -1) {
             povAngle += (robotState.onRedAlliance() ? 180 : 0);
             var robotRotationDeg = robotState.getOdometryPose().getRotation().getDegrees();
            //  //compare pov angle to robot rotation - robot rotation is ccw positive, but pov is cw positive.
             var povRobotMatched = -povAngle;
             if(povRobotMatched < -180) povRobotMatched += 360;

             invertFront = Math.abs(povRobotMatched - robotRotationDeg) > 90;
         }

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
