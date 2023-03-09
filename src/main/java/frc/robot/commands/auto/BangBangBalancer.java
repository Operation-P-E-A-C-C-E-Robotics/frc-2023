package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.DriveSignal;
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
        ModelBalancer.bangBangController(angle, DEADBAND, SPEED, driveTrain);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.set(DriveSignal.DEFAULT);
    }
}
