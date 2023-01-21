package frc.robot.commands.auto.paths;

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

    LTVUnicycleController controller = new LTVUnicycleController(0.02);
    private final DriveTrain driveTrain;
    private final Trajectory trajectory;
    private final Timer timer;
    private DifferentialDriveWheelSpeeds prevSpeeds;

    private double prevTime;
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
        timer = new Timer();
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        System.out.println("AHJKFLDA");
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
            System.out.println("HI");
            driveTrain.tankDriveVolts(0, 0);
            timer.reset();
            time = timer.get();
            prevTime = time;
            return;
        }

        System.out.println("time: " + time + " prev time: " + prevTime);
        System.out.println(robotState.getOdometryPose());
        System.out.println(trajectory.sample(time));

        //get chassis speeds from controller
        var chassis = controller.calculate(robotState.getOdometryPose(), trajectory.sample(time));
        System.out.println(chassis);
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
