package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.field.FieldConstants;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.Supersystem;

import java.lang.reflect.Field;
import java.util.ArrayList;

public class PlaceCubeStatemachine extends CommandBase {
    private final Supersystem supersystem;
    private final Target target;
    private final RobotState robotState;
    private State state = State.WAITING_FOR_DRIVETRAIN;
    private final Translation3d targetOffset = new Translation3d(-0.3, 0, 0.3); //TODO
    private Constants.SupersystemTolerance tolerance;
    private Translation3d targetLocation;

    @Deprecated
    public PlaceCubeStatemachine(Supersystem supersystem, Target target, RobotState robotState) {
        this.supersystem = supersystem;
        this.target = target;
        this.robotState = robotState;
        switch (target) {
            case LOW -> tolerance = Constants.SupersystemTolerance.PLACE_LOW;
            case MID -> tolerance = Constants.SupersystemTolerance.PLACE_MID;
            case HIGH -> tolerance = Constants.SupersystemTolerance.PLACE_HIGH;
        }
        addRequirements(supersystem.getRequirements());
    }

    @Override
    public void initialize(){
        state = State.WAITING_FOR_DRIVETRAIN;
        targetLocation = getScoreLocation().plus(targetOffset); //TODO update every loop?
    }
    
    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished(){
        return state == State.DONE;
    }

    /**
     * use the field map to find the nearest scoring location on the given level.
     * @return the location of the scoring location
     */
    private Translation3d getScoreLocation(){
        ArrayList<Translation3d> targets = switch(target){
            case LOW -> FieldConstants.Grids.lowTranslations3d;
            case MID -> FieldConstants.Grids.midCubeTranslations;
            case HIGH -> FieldConstants.Grids.highCubeTranslations;
        };
        return FieldConstants.Grids.getNearestNode(
                robotState.getRobotPose().getTranslation(),
                targets
        );
    }

    private enum State{
        WAITING_FOR_DRIVETRAIN,
        MOVING_ABOVE_TARGET,
        EJECT,
        DONE
    }

    public enum Target{
        LOW,
        MID,
        HIGH
    }
}
