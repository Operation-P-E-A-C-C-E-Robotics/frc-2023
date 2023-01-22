package frc.robot.commands.auto.paths;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveTrain;

//TODO experimental
public class PathFollower extends CommandBase{
    private final LTVUnicycleController controller;
    private final DriveTrain driveTrain;
    private final Trajectory trajectory;
    private final Timer timer;
    private final RobotState robotState;

    private DifferentialDriveWheelSpeeds prevSpeeds;
    private double prevTime;

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
                VecBuilder.fill(0.08,0.15,2), //maximum desired error tolerances (x meters, y meters, rotation rad)
                VecBuilder.fill(0.5,1.5), //maximum desired control effort (meters/second, rad/second)
                0.020 //discretion timestep (loop time) seconds
        );
        timer = new Timer();
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        var initialState = trajectory.sample(0);

        //get initial wheel speeds (to find initial acceleration)
        prevSpeeds = robotState.getDriveKinematics().toWheelSpeeds(new ChassisSpeeds(
            initialState.velocityMetersPerSecond,
            0,
            initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond
        ));

        //reset pid controllers
        driveTrain.resetVelocityDrive();

        //reset timer
        prevTime = -1;
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        var time = timer.get();
        var dt = time - prevTime; //time discretization step, whatevertf that means

        //account for time between init and execute.
        if (prevTime < 0){
            // System.out.println("HI");
            driveTrain.tankDriveVolts(0, 0);
            timer.reset();
            time = timer.get();
            prevTime = time;
            return;
        }

        //get chassis speeds from controller
        var chassis = controller.calculate(robotState.getOdometryPose(), trajectory.sample(time));

        //get wheel speeds
        var speeds = robotState.getDriveKinematics().toWheelSpeeds(chassis);

        //drive
        driveTrain.velocityDrive(speeds, prevSpeeds, dt);

        prevSpeeds = speeds;
        prevTime = time;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.tankDriveVolts(0, 0);
        timer.reset();
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

}
