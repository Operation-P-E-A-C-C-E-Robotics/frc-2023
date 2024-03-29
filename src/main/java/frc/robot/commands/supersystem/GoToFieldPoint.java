package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonicHRI;
import frc.robot.subsystems.Supersystem;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class GoToFieldPoint extends CommandBase {
    private final Supersystem supersystem;
    private Supplier<Translation3d> targetLocation;
    private double wristAngle;
    private final Constants.SupersystemTolerance tolerance;
    private final RobotState robotState;
    private final boolean stopWhenInTolerance;
    private final PhotonicHRI.PhotonicLingualElement outOfRange = RobotContainer.photonicHRI.blink(252, 90, 38, 0.1),
                                                    normal = RobotContainer.photonicHRI.blink(0, 255, 0, 0.1),
                                                    finished = RobotContainer.photonicHRI.setSolidColor(0, 255, 0),
                                                    interrupted = RobotContainer.photonicHRI.setSolidColor(255, 0, 0);

    /**
     * A command that moves the supersystem to a point on the field.
     * It will end when the supersystem is within tolerance of the target.
     */
    public GoToFieldPoint(Supersystem supersystem,
                          Supplier<Translation3d> targetLocation,
                          double wristAngle,
                          Constants.SupersystemTolerance tolerance,
                          RobotState robotState,
                          boolean stopWhenInTolerance) {
        this.supersystem = supersystem;
        this.targetLocation = targetLocation;
        this.wristAngle = wristAngle;
        this.tolerance = tolerance;
        this.robotState = robotState;
        this.stopWhenInTolerance = stopWhenInTolerance;
        addRequirements(supersystem.getRequirements());
    }

    public GoToFieldPoint(Supersystem supersystem,
                          Supplier<Translation3d> targetLocation,
                          Constants.SupersystemTolerance tolerance,
                          RobotState robotState,
                          boolean stopWhenInTolerance) {
        this.supersystem = supersystem;
        this.targetLocation = targetLocation;
        this.wristAngle = Math.PI/2;
        this.tolerance = tolerance;
        this.robotState = robotState;
        this.stopWhenInTolerance = stopWhenInTolerance;
        addRequirements(supersystem.getRequirements());
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        System.out.println(":)");
        var target = targetLocation.get();
        if(!robotState.inRangeOfTarget(new Translation2d(target.getX(), target.getY()))) {
            RobotContainer.photonicHRI.runElement(outOfRange);
            return;
        }
        RobotContainer.photonicHRI.runElement(normal);
        var robotRelative = robotState.fieldToDrivetrain(new Pose3d(target, new Rotation3d()));
        supersystem.setPlacePosition(robotRelative.getTranslation(), getTargetWristAngle());
    }

    @Override
    public void end(boolean interrupted){
        if(interrupted) RobotContainer.photonicHRI.runElement(this.interrupted);
        else RobotContainer.photonicHRI.runElement(finished);
    }

    public Rotation2d getTargetWristAngle(){
        var pivot = supersystem.getSupersystemState().getPivotAngle();
        if(Math.abs(pivot) < 0.1) return new Rotation2d();
        var wrist = pivot < 0 ? -wristAngle : wristAngle;
        return new Rotation2d(wrist);
    }

    @Override
    public boolean isFinished(){
        return supersystem.withinTolerance(tolerance) && stopWhenInTolerance;
    }

    public static Command targetVisionTape(Supersystem supersystem,
                                                  Translation3d initialTargetLocation,
                                                  Constants.SupersystemTolerance tolerance,
                                                  RobotState robotState,
                                                  Limelight armLimelight,
                                                  boolean stopWhenInTolerance) {
        Translation3d[] targetLocation = {initialTargetLocation}; //recall the target location from the last time we saw the target
        return new GoToFieldPoint(supersystem, () -> {
            //return the initial if we are too far away to see the target
            if(!supersystem.withinTolerance(Constants.SupersystemTolerance.TARGET_VISION)) return targetLocation[0];

            //return the initial if there is an error getting the target
            var target = robotState.getVisionTargetPose(initialTargetLocation.getZ());
            if(!target.isNormal()) return targetLocation[0];

            //return the x and y of the vision tape, and the initial z.
            var translation = target.get(new Pose3d(targetLocation[0], new Rotation3d()));
            targetLocation[0] = translation.getTranslation();
            return translation.getTranslation();
        }, tolerance, robotState, stopWhenInTolerance);
    }

    public static Command targetCone(Supersystem supersystem,
                                     Translation3d initialTargetLocation,
                                     Constants.SupersystemTolerance tolerance,
                                     RobotState robotState,
                                     boolean stopWhenInTolerance) {
        final Translation3d[] targetLocation = {initialTargetLocation}; //recall the target location from the last time we saw the target
        return new GoToFieldPoint(supersystem, () -> {
            //return the initial if we are too far away to see the target
            if(!supersystem.withinTolerance(Constants.SupersystemTolerance.TARGET_VISION)) return targetLocation[0];

            //return the initial if there is an error getting the target
            var target = robotState.getConePose();
            if(!target.isNormal()) return targetLocation[0];

            //return the x and y of the vision tape, and the initial z.
            var translation = target.get(new Pose3d(targetLocation[0], new Rotation3d()));
            translation = new Pose3d(translation.getX(), translation.getY(), initialTargetLocation.getZ(), translation.getRotation());
            targetLocation[0] = translation.getTranslation();
            return translation.getTranslation();
        }, tolerance, robotState, stopWhenInTolerance);
    }

    public static Command targetCube(Supersystem supersystem,
                                     Translation3d initialTargetLocation,
                                     Constants.SupersystemTolerance tolerance,
                                     RobotState robotState,
                                     boolean stopWhenInTolerance) {
        final Translation3d[] targetLocation = {initialTargetLocation}; //recall the target location from the last time we saw the target
        return new GoToFieldPoint(supersystem, () -> {
            //return the initial if we are too far away to see the target
            if(!supersystem.withinTolerance(Constants.SupersystemTolerance.TARGET_VISION)) return targetLocation[0];

            //return the initial if there is an error getting the target
            var target = robotState.getCubePose();
            if(!target.isNormal()) return targetLocation[0];

            //return the x and y of the vision tape, and the initial z.
            var translation = target.get(new Pose3d(targetLocation[0], new Rotation3d()));
            translation = new Pose3d(translation.getX(), translation.getY(), initialTargetLocation.getZ(), translation.getRotation());
            targetLocation[0] = translation.getTranslation();
            return translation.getTranslation();
        }, tolerance, robotState, stopWhenInTolerance);
    }
}
