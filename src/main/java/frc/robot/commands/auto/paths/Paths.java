package frc.robot.commands.auto.paths;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Odometry;
import frc.robot.subsystems.DriveTrain;
import static frc.robot.Constants.Auto.*;

import java.util.List;
/**
* will hold all paths the robot will follow
*/
public class Paths {
    DriveTrain driveTrain;
    Odometry odometry;
    DifferentialDriveVoltageConstraint constraint;
    TrajectoryConfig config;

    public Paths(Odometry odometry, DriveTrain driveTrain){
        this.driveTrain = driveTrain;
        this.odometry = odometry;
        constraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(kS, kV, kA),
            odometry.getKinematics(),
            AUTO_VOLTAGE_MAX
        );
        config = new TrajectoryConfig(
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
        ).setKinematics(odometry.getKinematics()).addConstraint(constraint);
    }
    public Command testPath(){
        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config);
        return createPathCommand(exampleTrajectory);
    }
    private Command createPathCommand(Trajectory trajectory){
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            odometry::getPose,
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            new SimpleMotorFeedforward(kS, kV, kA),
            odometry.getKinematics(),
            driveTrain::getWheelSpeeds,
            new PIDController(kP, kI, kD),
            new PIDController(kP, kI, kD),
            driveTrain::tankDriveVolts,
            driveTrain
        );
        return Commands.run(() -> odometry.resetPose(trajectory.getInitialPose()))
            .andThen(ramseteCommand)
            .andThen(() -> driveTrain.tankDrive(0,0));
    }
}
