package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.DriveSignal;
import frc.lib.util.PeaccyDriveHelper;
import frc.lib.util.Util;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveTrain;

public class BangBangBalancer extends CommandBase {
    private static final double DEADBAND = 1.5; //radians from center
    private static final double SPEED = 0.25; //drivetrain percentage
    private final RobotState robotState;
    private final DriveTrain driveTrain;
    private boolean drivingOn = true;
    private double initialHeading = 0;
    private boolean endWhenBalanced;

    public BangBangBalancer(DriveTrain drivetrain, RobotState robotState, boolean endWhenBalanced) {
        this.robotState = robotState;
        this.driveTrain = drivetrain;
        this.endWhenBalanced = endWhenBalanced;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivingOn = true;
        initialHeading = robotState.getPigeon().getHeading();
        robotState.getPigeon().zeroHeading();
    }
    static double maxBangBangSpeed = 0.55, minBangBangSpeed = 0.05;

    @Override
    public void execute() {
        double angle = robotState.getPigeon().getRoll();
        double headingError = robotState.getPigeon().getHeading() - initialHeading;
        if(Math.abs(angle) > 10) drivingOn = false;
        if(drivingOn){
            driveTrain.set(DriveSignal.tankDrive(0.4, 0.4, true));
        } else {
            double left, right;
            double error = Math.abs(angle);
            double speed = Util.interpolate(minBangBangSpeed, maxBangBangSpeed, error/20);
            if (angle > DEADBAND) {
                left = right = -speed;
                right += headingError * 0.001;
                left -= headingError * 0.001;
            } else if (angle < -DEADBAND) {
                left = right = speed;
                left += headingError * 0.001;
                right -= headingError * 0.001;
            } else {
                left = 0;
                right = 0;
            }
            driveTrain.set(DriveSignal.velocityDrive(left, right, true));        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.set(DriveSignal.DEFAULT);
    }

    @Override
    public boolean isFinished(){
        return endWhenBalanced && Util.inRange(robotState.getPigeon().getHeading(), DEADBAND);
    }
}
