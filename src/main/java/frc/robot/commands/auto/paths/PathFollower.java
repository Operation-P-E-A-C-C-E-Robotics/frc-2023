package frc.robot.commands.auto.paths;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Odometry;
import frc.robot.subsystems.DriveTrain;

//TODO experimental
public class PathFollower extends CommandBase{

    LTVUnicycleController controller = new LTVUnicycleController(0.02);
    private DriveTrain driveTrain;
    private Trajectory trajectory;
    private Timer timer;
    private DifferentialDriveWheelSpeeds prevSpeeds;

    private double prevTime;
    private Odometry odometry;

    /**
     * experimental path follower, to
     *  a. allow us to use an {@link LTVUnicycleController}, which
     *      I hear is better, and
     *  b. get more control over parameters and velocity control
     *      than with {@link edu.wpi.first.wpilibj2.command.RamseteCommand}
     * @param driveTrain drivetrain subsystem
     * @param trajectory trajectory to follow
     * @param odometry robot odometry.
     */
    public PathFollower(DriveTrain driveTrain, Trajectory trajectory, Odometry odometry){
        this.driveTrain = driveTrain;
        this.trajectory = trajectory;
        this.odometry = odometry;
        timer = new Timer();
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        var initialState = trajectory.sample(0);

        //get initial wheel speeds (to find initial acceleration)
        prevSpeeds = odometry.getKinematics().toWheelSpeeds(new ChassisSpeeds(
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
            driveTrain.tankDriveVolts(0, 0);
            prevTime = time;
            return;
        }

        //get chassis speeds from controller
        var chassis = controller.calculate(odometry.getPose(), trajectory.sample(time));

        //get wheel speeds
        var speeds = odometry.getKinematics().toWheelSpeeds(chassis);

        //drive
        driveTrain.velocityDrive(speeds, prevSpeeds, dt);

        prevSpeeds = speeds;
        prevTime = time;
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
