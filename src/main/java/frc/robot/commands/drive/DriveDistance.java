package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.DriveSignal;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
    private double initialHeading = 0, initialEncoder = 0;
    private final DriveTrain driveTrain;
    private double distance;
    private RobotState robotState;
    private boolean backwards;

    public DriveDistance(DriveTrain driveTrain, RobotState robotState, double distance){
        this.driveTrain = driveTrain;
        this.distance = distance;
        this.robotState = robotState;
        this.backwards = distance < 0;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize(){
        initialEncoder = driveTrain.getAverageEncoderDistance();
        initialHeading = robotState.getPigeon().getHeading();
    }

    @Override
    public void execute(){
        double left, right;
        double headingError = robotState.getPigeon().getHeading() - initialHeading;
        double speed = 0.4;
        if (!backwards) {
            left = right = speed;
            right -= headingError * 0.01;
            left += headingError * 0.01;
        } else {
            left = right = -speed;
            left -= headingError * 0.01;
            right += headingError * 0.01;
        }

        driveTrain.set(DriveSignal.velocityDrive(left, right, true));
    }

    @Override
    public boolean isFinished(){
        return Math.abs(driveTrain.getAverageEncoderDistance() - initialEncoder) > distance;
    }

    @Override
    public void end(boolean interrupted){
        driveTrain.set(DriveSignal.DEFAULT);
    }
}
