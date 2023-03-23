package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.DriveSignal;
import frc.lib.util.PeaccyDriveHelper;
import frc.lib.util.Util;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveTrain;

public class BangBangBalancer extends CommandBase {
    private static final double DEADBAND = 3; //radians from center
    private static final double SPEED = 0.25; //drivetrain percentage
    private final RobotState robotState;
    private final DriveTrain driveTrain;
    private boolean drivingOn = true;
    private double initialHeading = 0;
    private boolean endWhenBalanced;

    /**
     * WELCOME to the wonderful world of code written right before + during a competition.
     * @param drivetrain the drivetrain
     * @param robotState the robot state
     * @param endWhenBalanced whether to end the command when the robot is balanced
     */
    public BangBangBalancer(DriveTrain drivetrain, RobotState robotState, boolean endWhenBalanced) {
        this.robotState = robotState;
        this.driveTrain = drivetrain;
        this.endWhenBalanced = endWhenBalanced;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize(){
        drivingOn = true; //THE ROBOT NEEDS TO DRIVE ON FIRST
        initialHeading = robotState.getPigeon().getHeading();
        robotState.getPigeon().zeroHeading();
    }
    static double maxBangBangSpeed = 0.55, minBangBangSpeed = 0.05;

    @Override
    public void execute() {
        //GET THE PIGEON ANGLES:
        double angle = robotState.getPigeon().getRoll();
        double headingError = robotState.getPigeon().getHeading() - initialHeading;

        if(Math.abs(angle) > 10) drivingOn = false; //if we're tilted more than 10 degrees, start balancing
        if(drivingOn){
            //to drive on, drive at 0.4 speed (TODO maybe change to low gear if we fix the issues with that)
            driveTrain.set(DriveSignal.tankDrive(0.4, 0.4, true));
        } else {
            double left, right;
            double error = Math.abs(angle); //Bad robot isn't balanced yet

            //interpolate between the min and max velocity to stop the robot from going too fast
            double speed = Util.interpolate(minBangBangSpeed, maxBangBangSpeed, error/20);
            if (angle > DEADBAND) {
                left = right = -speed;
                //adjust for heading error to keep the robot straight
                right += headingError * 0.001;
                left -= headingError * 0.001;
            } else if (angle < -DEADBAND) {
                left = right = speed;
                left += headingError * 0.001;
                right -= headingError * 0.001;
            } else {
                driveTrain.set(DriveSignal.DEFAULT);
                return;
            }
            driveTrain.set(DriveSignal.velocityDrive(left, right, true));
        }
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
