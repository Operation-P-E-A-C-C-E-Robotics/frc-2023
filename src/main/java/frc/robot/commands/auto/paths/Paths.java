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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.RobotState_old;
import frc.robot.subsystems.DriveTrain;

import java.util.List;

import static frc.robot.Constants.Auto.*;
/**
* will hold all paths the robot will follow
*/
public class Paths {
    DriveTrain driveTrain;
    RobotState_old robotState;
    DifferentialDriveVoltageConstraint constraint;
    TrajectoryConfig config;

    /**
    * class to hold all the paths the robot will follow
    */
    public Paths(RobotState_old robotState, DriveTrain driveTrain){
        this.driveTrain = driveTrain;
        this.robotState = robotState;
        constraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(kS, kV, kA),
            robotState.getDriveKinematics(),
            AUTO_VOLTAGE_MAX
        );
        config = new TrajectoryConfig(
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
        ).setKinematics(robotState.getDriveKinematics()).addConstraint(constraint);
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

    /**
    * create a ramsete command from a trajectory
    * @param trajectory the trajectory to follow.
    * @return a ramsete command to follow the path
    */
    private Command createPathCommand(Trajectory trajectory){
        //LTVUnicycleController what = new LTVUnicycleController
        RamseteCommand ramseteCommand = new RamseteCommand(
            trajectory,
            robotState::getPose,
            new RamseteController(RAMSETE_B, RAMSETE_ZETA),
            new SimpleMotorFeedforward(kS, kV, kA),
            robotState.getDriveKinematics(),
            driveTrain::getWheelSpeeds,
            new PIDController(kP, kI, kD),
            new PIDController(kP, kI, kD),
            driveTrain::tankDriveVolts,
            driveTrain
        );
        return ramseteCommand.andThen(() -> driveTrain.tankDrive(0,0));
    }
}
