package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Supersystem;

public class GoToFieldPoint extends CommandBase {
    private final Supersystem supersystem;
    private Translation3d targetLocation;
    private final Constants.SupersystemTolerance tolerance;
    private final RobotState robotState;
    private final boolean stopWhenInTolerance;

    /**
     * A command that moves the supersystem to a point on the field.
     * It will end when the supersystem is within tolerance of the target.
     */
    public GoToFieldPoint(Supersystem supersystem,
                          Translation3d targetLocation,
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

    public void updateTargetLocation(Translation3d targetLocation){
        this.targetLocation = targetLocation;
    }

    @Override
    public void execute(){
        if(!robotState.inRangeOfTarget(new Translation2d(targetLocation.getX(), targetLocation.getY()))) return;
        var drivetrainRelativeTarget = robotState.fieldToDrivetrain(new Pose3d(targetLocation, new Rotation3d()));
        supersystem.setSupersystemPosition(drivetrainRelativeTarget.getTranslation());
    }

    @Override
    public boolean isFinished(){
        return supersystem.withinTolerance(tolerance) && stopWhenInTolerance;
    }
}
