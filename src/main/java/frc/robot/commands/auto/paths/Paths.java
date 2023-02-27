package frc.robot.commands.auto.paths;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.Util;
import frc.lib.safety.Value;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveTrain;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import static frc.robot.Constants.DriveTrain.*;
/**
* will hold all paths the robot will follow
*/
public class Paths {
    DriveTrain driveTrain;
    //RobotState robotState;
    DifferentialDriveVoltageConstraint constraint;
    TrajectoryConfig config;

    /**
    * class to hold all the paths the robot will follow
    */
    public Paths(RobotState robotState, DriveTrain driveTrain){
        this.driveTrain = driveTrain;
        //this.robotState = robotState;
        constraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(kS, kV_LINEAR, kA_LINEAR),
            DRIVE_KINEMATICS,
            AUTO_VOLTAGE_MAX
        );
        config = new TrajectoryConfig(
            AUTO_MAX_SPEED_METERS_PER_SECOND,
            AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED
        ).setKinematics(DRIVE_KINEMATICS).addConstraint(constraint);
    }

    public Command ohshitpath(RobotState robotState) { //TODO Refactor Name
        Trajectory ohshit = new Trajectory();
        try{
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/Unnamed.wpilib.json");
            ohshit = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException e){
            System.out.println("OHHHH SHIT.");
        }
        return createPathCommand(ohshit, robotState);
    }

    public Command testPath(RobotState robotState){
        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0.8, 0.1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(1.5, 0, Rotation2d.fromDegrees(0)),
            // Pass config
            config);
        return createPathCommand(exampleTrajectory, robotState);
    }

    private Pose2d current = new Pose2d();
    private Pose2d target = new Pose2d();

    public Value<PathFollower> driveToConeCommand(RobotState state, DriveTrain driveTrain){
        current = state.getOdometryPose();
        var cone = state.getConePose();
        if(!cone.isNormal()) return Value.none();
        target = Util.toPose2d(cone.get(new Pose3d()));
        return new Value<>(new PathFollower(driveTrain, TrajectoryGenerator.generateTrajectory(
            current,
            List.of(/*middle.getTranslation()*/),
            target.plus(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180))),  
            config.setReversed(true)
            ),
        state));
    }

    /**
    * create a ramsete command from a trajectory
    * @param trajectory the trajectory to follow.
    * @return a ramsete command to follow the path
    */
    private Command createPathCommand(Trajectory trajectory, RobotState robotState){
        return new PathFollower(driveTrain, trajectory, robotState);
    }
}
