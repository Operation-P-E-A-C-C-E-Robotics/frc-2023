package frc.robot.commands.auto.paths;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.DriveSignal;
import frc.robot.DashboardManager;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.DriveTrain.DRIVE_KINEMATICS;

//TODO experimental
public class PathFollower extends CommandBase{
    private final LTVUnicycleController controller;
    private final DriveTrain driveTrain;
    private final Trajectory trajectory;
    private final Timer timer;
    private final RobotState robotState;

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
    public PathFollower(DriveTrain driveTrain, Trajectory trajectory, RobotState robotState){
        this.driveTrain = driveTrain;
        this.trajectory = trajectory;
        this.robotState = robotState;
        controller = new LTVUnicycleController(
                VecBuilder.fill(0.1,0.2,0.5), //maximum desired error tolerances (x meters, y meters, rotation rad)
                VecBuilder.fill(0.5,0.3), //maximum desired control effort (meters/second, rad/second)
                0.020 //discretion timestep (loop time) seconds
        );
        timer = new Timer();
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        //reset pid controllers
        driveTrain.resetVelocityDrive();
        // if we reset the odometry, the path will follow as it is drawn
        // if we don't reset the odometry, the controller will try to compensate for error in previous paths /
        // error in starting position, which doesn't always work the greatest, but *could* improve accuracy.
       robotState.resetOdometry(trajectory.getInitialPose()); //TODO do we want this? (see above)
        DashboardManager.getInstance().drawTrajectory(trajectory);
    }

    @Override
    public void execute() {
        var time = timer.get();

        //get chassis speeds from controller
        var chassis = controller.calculate(robotState.getOdometryPose(), trajectory.sample(time));

        //get wheel speeds
        var speeds = DRIVE_KINEMATICS.toWheelSpeeds(chassis);
        speeds = new DifferentialDriveWheelSpeeds(speeds.rightMetersPerSecond, speeds.leftMetersPerSecond);

        //drive
        driveTrain.set(DriveSignal.velocityDrive(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond, true));
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.set(DriveSignal.DEFAULT);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

}
