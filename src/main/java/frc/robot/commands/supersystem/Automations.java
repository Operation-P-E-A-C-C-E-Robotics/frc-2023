package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.field.FieldConstants;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.endeffector.DropGamepiece;
import frc.robot.commands.endeffector.SpitGamepiece;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Supersystem;

import java.util.ArrayList;

/**
 * A class to hold all the little automation commands that are mostly chains of other commands.
 */
public class Automations {
    public static Translation3d cubePlaceOffset = new Translation3d(0.3, 0, 0.3); //TODO
    public static Translation3d conePrePlaceOffset = new Translation3d(0, 0, 0.3); //TODO
    public static Translation3d conePlaceOffset = new Translation3d(0, 0, 0); //TODO
    private final Supersystem supersystem;
    private final RobotState robotState;
    private final EndEffector endEffector;

    public Automations(Supersystem supersystem, RobotState robotState, EndEffector endEffector){
        this.supersystem = supersystem;
        this.robotState = robotState;
        this.endEffector = endEffector;
    }

    /**
     * A command that moves the supersystem into position to place a cube.
     * @param level the level to place the cube on
     * @return a command that moves the supersystem into position to place a cube
     */
    public Command goToCubePosition(PlaceLevel level){
        var tolerance = switch(level){
            case HIGH -> Constants.SupersystemTolerance.PLACE_HIGH;
            case MID -> Constants.SupersystemTolerance.PLACE_MID;
            case LOW -> Constants.SupersystemTolerance.PLACE_LOW;
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
                tolerance,
                robotState,
                false
        );
    }

    public Command placeCube(PlaceLevel level){
        return goToCubePosition(level).raceWith(new SpitGamepiece(endEffector));
    }

    public Command placeConeNoVision(PlaceLevel level){
        var tolerance = switch(level){
            case HIGH -> Constants.SupersystemTolerance.PLACE_HIGH;
            case MID -> Constants.SupersystemTolerance.PLACE_MID;
            case LOW -> Constants.SupersystemTolerance.PLACE_LOW;
        };
        ArrayList<Translation3d> scoreLocations = switch(level){
            case HIGH -> FieldConstants.Grids.highConeTranslations;
            case MID -> FieldConstants.Grids.midConeTranslations;
            case LOW -> FieldConstants.Grids.lowTranslations3d;
        };
        var goToPrePlace = new GoToFieldPoint(
                supersystem,
                () -> FieldConstants.Grids.getNearestNode(robotState.getRobotPose().getTranslation(), scoreLocations)
                        .plus(conePrePlaceOffset),
                tolerance,
                robotState,
                true
        );
        var goToPlace = new GoToFieldPoint(
                supersystem,
                () -> FieldConstants.Grids.getNearestNode(robotState.getRobotPose().getTranslation(), scoreLocations)
                        .plus(conePlaceOffset),
                tolerance,
                robotState,
                true
        );
        return goToPrePlace.andThen(goToPlace.raceWith(new DropGamepiece(endEffector)));
    }

    public Command pickUpConeFloor(){
        //TODO OOOOOOO
        return Setpoints.goToSetpoint(Setpoints.intakeFloor, supersystem, Constants.SupersystemTolerance.INTAKE_GROUND);
    }

    //TODO: will this work?????????????????????????
    public Command place(PlaceLevel level){
        return (placeConeNoVision(level).unless(() -> endEffector.getState() != EndEffector.IntakeState.HAS_CONE)).alongWith(
                placeCube(level).unless(() -> endEffector.getState() != EndEffector.IntakeState.HAS_CUBE));
    }

    public enum PlaceLevel{
        HIGH,
        MID,
        LOW
    }
}
