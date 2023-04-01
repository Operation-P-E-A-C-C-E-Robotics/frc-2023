package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.field.FieldConstants;
import frc.lib.safety.Value;
import frc.robot.Constants.SupersystemTolerance;
import frc.robot.Kinematics;
import frc.robot.RobotState;
import frc.robot.commands.endeffector.DropGamepiece;
import frc.robot.commands.endeffector.SpitGamepiece;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Supersystem;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

public class Automations {
    public static Translation3d cubePlaceOffset = new Translation3d(0, 0, 0.35); //TODO
    public static Translation3d conePrePlaceOffset = new Translation3d(-0, 0, 0.14); //TODO
    public static Translation3d conePlaceOffset = new Translation3d(-0, 0, 0.00); //TODO
    private final Supersystem supersystem;
    private final RobotState robotState;
    private final EndEffector endEffector;
    private final Setpoints setpoints;

    /**
     * A class to hold all the little automation commands that are mostly chains of other commands.
     * @param supersystem the supersystem to use
     * @param robotState the robot state to use
     * @param endEffector the end effector to use
     * @param setpoints the setpoints to use
     */
    public Automations(Supersystem supersystem, RobotState robotState, EndEffector endEffector, Setpoints setpoints){
        this.supersystem = supersystem;
        this.robotState = robotState;
        this.endEffector = endEffector;
        this.setpoints = setpoints;
    }

    /**
     * A command that moves the supersystem into position to place a cube.
     * @param level the level to place the cube on
     * @return a command that moves the supersystem into position to place a cube
     */
    public Command goToCubePosition(PlaceLevel level){
        var tolerance = switch(level){
            case HIGH -> SupersystemTolerance.PLACE_HIGH;
            case MID -> SupersystemTolerance.PLACE_MID;
            case LOW -> SupersystemTolerance.PLACE_LOW;
        };
        ArrayList<Translation3d> scoreLocations = switch(level){
            case HIGH -> FieldConstants.Grids.highCubeTranslations;
            case MID -> FieldConstants.Grids.midCubeTranslations;
            case LOW -> FieldConstants.Grids.lowTranslations3d;
        };
        return new GoToFieldPoint(
                supersystem,
                () -> FieldConstants.Grids.getNearestNode(robotState.getRobotPose().getTranslation(), scoreLocations)
                        .plus(cubePlaceOffset),
                135,
                tolerance,
                robotState,
                false
        );
    }

    /**
     * A command to place a cube (moves into position and then spits it out)
     * @param level the level to place the cube on
     * @return a command to place a cube
     */
    public Command placeCube(PlaceLevel level){
        return goToCubePosition(level)
                 .raceWith(new SpitGamepiece(
                         endEffector,
                         () -> supersystem.withinTolerance(SupersystemTolerance.forLevel(level))
                 ));
    }

    /**
     * place a cone with odometry. 3 step process: move to a position above the peg,
     * move down so the cone is on the peg, then drop.
     * @param level the level to place the cone on
     * @return a command to place a cone
     */
    public Command placeConeNoVision(PlaceLevel level){
        var tolerance = SupersystemTolerance.forLevel(level);
        ArrayList<Translation3d> scoreLocations = switch(level){
            case HIGH -> FieldConstants.Grids.highConeTranslations;
            case MID -> FieldConstants.Grids.midConeTranslations;
            case LOW -> FieldConstants.Grids.lowTranslations3d;
        };
        var goToPrePlace = new GoToFieldPoint(
                supersystem,
                () -> FieldConstants.Grids.getNearestNode(robotState.getRobotPose().getTranslation(), scoreLocations)
                        .plus(conePrePlaceOffset),
                Units.degreesToRadians(100),
                SupersystemTolerance.PRE_PLACE,
                robotState,
                true
        ).withTimeout(3);
        var goToPlace = new GoToFieldPoint(
                supersystem,
                () -> FieldConstants.Grids.getNearestNode(robotState.getRobotPose().getTranslation(), scoreLocations)
                        .plus(conePlaceOffset),
                Units.degreesToRadians(125),
                tolerance,
                robotState,
                true
        ).withTimeout(1);
        return goToPrePlace.andThen(new WaitCommand(0.5),goToPlace).andThen(new DropGamepiece(endEffector, () -> true));
    }

    private static final double SENSITIVITY = 0.02;

    public Command placeConeDriverAssist(PlaceLevel level, DoubleSupplier xOffset, DoubleSupplier yOffset, DoubleSupplier zOffset){
        var tolerance = SupersystemTolerance.forLevel(level);
        ArrayList<Translation3d> scoreLocations = switch(level){
            case HIGH -> FieldConstants.Grids.highConeTranslations;
            case MID -> FieldConstants.Grids.midConeTranslations;
            case LOW -> FieldConstants.Grids.lowTranslations3d;
        };
        AtomicReference<Double> xRef = new AtomicReference<>(0.0), yRef = new AtomicReference<>(0.0), zRef = new AtomicReference<>(0.0);
        return new GoToFieldPoint(
            supersystem,
            () -> {
                var gridLocation = FieldConstants.Grids.getNearestNode(robotState.getRobotPose().getTranslation(), scoreLocations);
                return gridLocation.plus(new Translation3d(
                    xRef.getAndUpdate(v -> v + xOffset.getAsDouble() * SENSITIVITY),
                    yRef.getAndUpdate(v -> v + yOffset.getAsDouble() * SENSITIVITY),
                    zRef.getAndUpdate(v -> v + zOffset.getAsDouble() * SENSITIVITY)
                ));
            },
            tolerance,
            robotState,
            false
        );
    }

    public Command smartZero(){
        return new RunCommand(() -> supersystem.setArm(0), supersystem.getRequirements()).withTimeout(0.5)
            .andThen(new WaitCommand(0.5))
            .andThen(new RunCommand(() -> supersystem.setPivot(new Rotation2d()), supersystem.getRequirements())).withTimeout(0.5)
            .andThen(setpoints.goToSetpoint(Setpoints.zero));
    }

    /**
     * @return a command to look for a cone on the floor, and if it's near enough reach out and grab it.
     */
    public Command targetConeFloor(){
        return new RunCommand(() -> {
            var conePose = robotState.getConePose();
            targetPose(conePose);
        }, supersystem.getRequirements());
    }

    /**
     * @return a command to look for a cube on the floor, and if it's near enough reach out and grab it.
     */
    public Command targetCubeFloor(){
        return new RunCommand(() -> {
            var cubePose = robotState.getCubePose();
            targetPose(cubePose);
        }, supersystem.getRequirements());
    }


    /**
     * Not a command! a helper to be run from other commands.
     * Points the turret at the target, and if it's close enough, moves the whole supersystem to the target.
     * @param targetPose the target pose
     */
    private void targetPose(Value<Pose3d> targetPose) {
        if(!targetPose.isNormal()) return;

        //get the kinematics for the pose
        var pose = targetPose.get(new Pose3d());
        var kinematics = Kinematics.inverseKinematicsFromPlacePoint(pose.getTranslation(), supersystem.getSupersystemState().getWristAngle());
        var angle = kinematics.getTurretAngle(); // just the turret angle of the kinematics

        //if the target is fairly close, just go grab it.
        if(kinematics.getArmExtension() < 0.7){
            supersystem.setSupersystemState(kinematics);
            return;
        }

        //otherwise, only move the turret
        supersystem.setTurret(new Rotation2d(angle));
    }

    public Command pickUpConeFloor(){
        //TODO OOOOOOO
        return setpoints.goToSetpoint(Setpoints.intakeFloor, SupersystemTolerance.INTAKE_GROUND)
                .andThen(targetConeFloor(), new RunCommand(() -> endEffector.setClaw(false), endEffector));
    }

    public Command pickUpCubeFloor(){
        //TODO OOOOOOO
        return setpoints.goToSetpoint(Setpoints.intakeFloor, SupersystemTolerance.INTAKE_GROUND)
                .andThen(targetCubeFloor());
    }

    //TODO: will this work?????????????????????????
    public Command place(PlaceLevel level){
        return (placeConeNoVision(level).unless(() -> endEffector.getState() != EndEffector.IntakeState.HAS_CONE)).andThen(
                placeCube(level).unless(() -> endEffector.getState() != EndEffector.IntakeState.HAS_CUBE));
    }

    public enum PlaceLevel{
        HIGH,
        MID,
        LOW
    }
}
