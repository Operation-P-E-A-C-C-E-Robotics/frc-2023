package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.subsystems.Supersystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultStatemachine extends CommandBase {
    private final double TARGET_ROTATION = 0, PICKUP_ROTATION = Math.PI;
    private final Kinematics.SupersystemState restingState = new Kinematics.SupersystemState(0, 0, Constants.Arm.MIN_EXTENSION, Math.PI/2);
    private final Translation3d tiltTranslationForward = new Translation3d(0.3, 0, Constants.Arm.MIN_EXTENSION);
    private final BooleanSupplier tiltTowardsTargetSupplier, tiltTowardsPickupSupplier;
    private final Supersystem supersystem;
    private final DoubleSupplier robotHeading;
    private boolean automate = true;
    private State state = State.RESTING;

    /**
     * Default statemachine for the supersystem.
     * Hold the arm in an upright position, fully retracted. Depending on
     * the level of automation and position of the field, tilt the arm and
     * turret towards side of the field that is of interest.
     * @param supersystem
     */
    public DefaultStatemachine(Supersystem supersystem, BooleanSupplier tiltTowardsTargetSupplier, BooleanSupplier tiltTowardsPickupSupplier, DoubleSupplier robotHeading) {
        addRequirements(supersystem.getRequirements());
        this.supersystem = supersystem;
        this.tiltTowardsTargetSupplier = tiltTowardsTargetSupplier;
        this.tiltTowardsPickupSupplier = tiltTowardsPickupSupplier;
        this.robotHeading = robotHeading;
    }

    public void enableTiltTowardsTargets(boolean enabled){
        automate = enabled;
    }

    @Override
    public void initialize() {
        state = State.RESTING;
    }

    @Override
    public void execute(){
        state = updateState();
        handleStateTransition(state);
    }

    private void handleStateTransition(State newState){
        var heading = robotHeading.getAsDouble();
        switch(state){
            case RESTING:
                supersystem.setSupersystemState(restingState);
                break;
            case TILT_TOWARDS_TARGETS:
                supersystem.setSupersystemPosition(Util.rotateBy(
                        tiltTranslationForward,
                        new Rotation3d(0,0,TARGET_ROTATION-heading)
                ));
                break;
            case TILT_TOWARDS_PICKUP:
                supersystem.setSupersystemPosition(Util.rotateBy(
                        tiltTranslationForward,
                        new Rotation3d(0,0,PICKUP_ROTATION-heading)
                ));
                break;
        }
    }

    private State updateState(){
        if(!automate) {
            return State.RESTING;
        }
        if(tiltTowardsPickupSupplier.getAsBoolean()){
            return State.TILT_TOWARDS_PICKUP;
        } else if (tiltTowardsTargetSupplier.getAsBoolean()) {
            return State.TILT_TOWARDS_TARGETS;
        } else {
            return State.RESTING;
        }
    }

    private enum State{
        RESTING,
        TILT_TOWARDS_TARGETS,
        TILT_TOWARDS_PICKUP
    }
}
