package frc.robot.commands.drive;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveTrain;

//TODO experimental
public class HoldDrivetrain extends CommandBase{

    private final double dt;
    LTVUnicycleController controller;
    private final DriveTrain driveTrain;
    private DifferentialDriveWheelSpeeds prevSpeeds;
    private final RobotState robotState;
    private Pose2d target;

    /**
     * experimental path follower, to
     *  a. allow us to use an {@link LTVUnicycleController}, which
     *      I hear is better, and
     *  b. get more control over parameters and velocity control
     *      than with {@link edu.wpi.first.wpilibj2.command.RamseteCommand}
     * @param driveTrain drivetrain subsystem
     * @param trajectory trajectory to follow
     * @param robotState robot odometry.
     */
    public HoldDrivetrain(DriveTrain driveTrain, Trajectory trajectory, RobotState robotState, double dt){
        this.driveTrain = driveTrain;
        this.robotState = robotState;
        this.dt = dt;
        controller = new LTVUnicycleController(dt);
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        //get initial wheel speeds (to find initial acceleration)
        prevSpeeds = new DifferentialDriveWheelSpeeds(0,0);

        this.target = robotState.getOdometryPose();
        //reset pid controllers
        driveTrain.resetVelocityDrive();
    }

    @Override
    public void execute() {
        //get chassis speeds from controller
        var chassis = controller.calculate(robotState.getOdometryPose(), new State(0,0,0,target,0));

        //get wheel speeds
        var speeds = Constants.DriveTrain.DRIVE_KINEMATICS.toWheelSpeeds(chassis);

        //drive
        driveTrain.velocityDrive(speeds, prevSpeeds, dt);

        prevSpeeds = speeds;
    }


    @Override
    public void end(boolean interrupted) {
        driveTrain.tankDriveVolts(0, 0);
    }
    @Override
    public boolean isFinished() {
        return false;
    }

}
