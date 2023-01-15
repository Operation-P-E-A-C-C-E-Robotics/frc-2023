package frc.robot.commands.auto.paths;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Odometry;
import frc.robot.subsystems.DriveTrain;

//TODO experimental
public class PathFollower extends CommandBase{

    LTVUnicycleController controller = new LTVUnicycleController(0.02);
    private DriveTrain driveTrain;
    private Trajectory trajectory;
    private Timer timer;
    private DifferentialDriveWheelSpeeds prevSpeeds;

    private SimpleMotorFeedforward feedforward;
    private PIDController leftController, rightController;

    private double prevTime;
    private Odometry odometry;

    public PathFollower(DriveTrain driveTrain, Trajectory trajectory, Odometry odometry){
        this.driveTrain = driveTrain;
        this.trajectory = trajectory;
        this.odometry = odometry;
        timer = new Timer();
    }

    @Override
    public void initialize() {
        var initialState = trajectory.sample(0);
        timer.reset();
        prevSpeeds = odometry.getKinematics().toWheelSpeeds(new ChassisSpeeds(
            initialState.velocityMetersPerSecond,
            0,
            initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond
        ));
        driveTrain.resetVelocityDrive();
        prevTime = -1;
        timer.start();
    }

    @Override
    public void execute() {
        var time = timer.get();
        var dt = time - prevTime;

        if (prevTime < 0){
            driveTrain.tankDriveVolts(0, 0);
            prevTime = time;
            return;
        }

        var chassis = controller.calculate(odometry.getPose(), trajectory.sample(time));

        var speeds = odometry.getKinematics().toWheelSpeeds(chassis);

        driveTrain.velocityDrive(speeds, prevSpeeds, dt);

        prevSpeeds = speeds;
        prevTime = time;
    }


    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return false;
    }

}
