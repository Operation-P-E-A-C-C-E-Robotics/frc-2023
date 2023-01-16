package frc.robot.commands.drive;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Odometry;
import frc.robot.subsystems.DriveTrain;

//TODO experimental
public class HoldDrivetrain extends CommandBase{

    LTVUnicycleController controller = new LTVUnicycleController(0.02);
    private final DriveTrain driveTrain;
    private DifferentialDriveWheelSpeeds prevSpeeds;
    private final Odometry odometry;
    private Pose2d target;

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
    public HoldDrivetrain(DriveTrain driveTrain, Trajectory trajectory, Odometry odometry){
        this.driveTrain = driveTrain;
        this.odometry = odometry;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        //get initial wheel speeds (to find initial acceleration)
        prevSpeeds = new DifferentialDriveWheelSpeeds(0,0);

        this.target = odometry.getPose();
        //reset pid controllers
        driveTrain.resetVelocityDrive();
    }

    @Override
    public void execute() {
        //get chassis speeds from controller
        var chassis = controller.calculate(odometry.getPose(), new State(0,0,0,target,0));

        //get wheel speeds
        var speeds = odometry.getKinematics().toWheelSpeeds(chassis);

        //drive
        driveTrain.velocityDrive(speeds, prevSpeeds, 0.02);

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
