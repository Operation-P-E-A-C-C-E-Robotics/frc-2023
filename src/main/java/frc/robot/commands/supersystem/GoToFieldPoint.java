package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Supersystem;

import java.util.function.Supplier;

public class GoToFieldPoint extends CommandBase {
    private final Supersystem supersystem;
    private Supplier<Translation3d> targetLocation;
    private final Constants.SupersystemTolerance tolerance;
    private final RobotState robotState;
    private final boolean stopWhenInTolerance;

    /**
     * A command that moves the supersystem to a point on the field.
     * It will end when the supersystem is within tolerance of the target.
     */
    public GoToFieldPoint(Supersystem supersystem,
                          Supplier<Translation3d> targetLocation,
                          Constants.SupersystemTolerance tolerance,
                          RobotState robotState,
                          boolean stopWhenInTolerance) {
        this.supersystem = supersystem;
        this.targetLocation = targetLocation;
        this.tolerance = tolerance;
        this.robotState = robotState;
        this.stopWhenInTolerance = stopWhenInTolerance;
        addRequirements(supersystem.getRequirements());
    }

    @Override
    public void execute(){
        var target = targetLocation.get();
        if(!robotState.inRangeOfTarget(new Translation2d(target.getX(), target.getY()))) return;
        supersystem.setPlacePositionFieldRelative(target, robotState);
    }

    @Override
    public boolean isFinished(){
        return supersystem.withinTolerance(tolerance) && stopWhenInTolerance;
    }
}
