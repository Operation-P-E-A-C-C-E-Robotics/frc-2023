package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveTrain;

public class BangBangBalancer extends CommandBase {
    private static final double DEADBAND = 0.1; //radians from center
    private static final double SPEED = 0.3; //drivetrain percentage
    private final RobotState robotState;
    private final DriveTrain driveTrain;

    public BangBangBalancer(DriveTrain drivetrain, RobotState robotState) {
        this.robotState = robotState;
        this.driveTrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double angle = robotState.getRobotPose().getRotation().getY();
        double left, right;
        if (angle > DEADBAND) {
            left = SPEED;
            right = -SPEED;
        } else if (angle < -DEADBAND) {
            left = -SPEED;
            right = SPEED;
        } else {
            left = 0;
            right = 0;
        }
        driveTrain.tankDrive(left, right);

    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.tankDrive(0, 0); //TODO maybe make this some hold position command
    }
}
