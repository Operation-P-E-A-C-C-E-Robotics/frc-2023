package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Limelight;
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
        var robotRelative = robotState.fieldToDrivetrain(new Pose3d(target, new Rotation3d()));
        supersystem.setPlacePosition(robotRelative.getTranslation(), getWristAngle());
    }

    public Rotation2d getWristAngle(){
        var pivot = supersystem.getSupersystemState().getPivotAngle();
        if(Math.abs(pivot) < 0.1) return new Rotation2d();
        var wrist = pivot < 0 ? -Math.PI/2 : Math.PI/2;
        wrist -= pivot;
        return new Rotation2d(wrist);
    }

    @Override
    public boolean isFinished(){
        return supersystem.withinTolerance(tolerance) && stopWhenInTolerance;
    }

    //TODO this is very wrong but i'm tired i'll fix it later
    public static Command targetWithVisionCommand(Supersystem supersystem,
                                                  Translation3d initialTargetLocation,
                                                  Constants.SupersystemTolerance tolerance,
                                                  RobotState robotState,
                                                  Limelight armLimelight,
                                                  boolean stopWhenInTolerance) {
        return new GoToFieldPoint(supersystem, () -> {
            //return the initial if we are too far away to see the target
            if(!supersystem.withinTolerance(Constants.SupersystemTolerance.TARGET_VISION)) return initialTargetLocation;

            //return the initial if there is an error getting the target
            var target = robotState.getVisionTargetPose(initialTargetLocation.getZ());
            if(!target.isNormal()) return initialTargetLocation;

            //return the x and y of the vision tape, and the initial z.
            var translation = target.get(new Pose3d(initialTargetLocation, new Rotation3d()));
            return translation.getTranslation();
        }, tolerance, robotState, stopWhenInTolerance);
    }
}
