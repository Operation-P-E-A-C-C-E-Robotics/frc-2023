package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.field.FieldConstants;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Supersystem;

import java.lang.reflect.Field;
import java.util.ArrayList;

/**
 * A class to hold all the little automation commands that are mostly chains of other commands.
 */
public class Automations {
    public static Translation3d cubePlaceOffset = new Translation3d(-0.3, 0, 0.3); //TODO
    public static Translation3d conePrePlaceOffset = new Translation3d(-0.3, 0, 0.3); //TODO
    public static Translation3d conePlaceOffset = new Translation3d(-0.3, 0, 0.3); //TODO

    /**
     * A command that moves the supersystem into position to place a cube.
     * @param supersystem the supersystem to move
     * @param level the level to place the cube on
     * @param robotState the robot state
     * @return a command that moves the supersystem into position to place a cube
     */
    public static Command goToCubePosition(Supersystem supersystem, PlaceLevel level, RobotState robotState){
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

    public static Command placeCube(Supersystem supersystem, PlaceLevel level, RobotState robotState){
        return goToCubePosition(supersystem, level, robotState).raceWith(/*TODO eject when ready command*/);
    }

    public static Command placeConeNoVision(Supersystem supersystem, PlaceLevel level, RobotState robotState){
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
                false
        );
        return goToPrePlace.andThen(goToPlace.raceWith(/*TODO eject when ready command*/));
    }

    public enum PlaceLevel{
        HIGH,
        MID,
        LOW
    }
}
