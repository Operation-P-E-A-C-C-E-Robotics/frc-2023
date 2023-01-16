package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Kinematics.SupersystemState;
import frc.robot.subsystems.Supersystem;

//not sure where this class should go yet.
//basing this file's structure off of 254 2019
public class SupersystemStateMachine {
    private final Supersystem supersystem;
    private SupersystemState state = STRAIGHT_UP;
    public SupersystemStateMachine(Supersystem supersystem){
        this.supersystem = supersystem;
    }

    
    
    //TODO position setpoints:
    private static final SupersystemState
            STRAIGHT_UP = new SupersystemState(0,0,0,0),
            FOLDED = new SupersystemState(0,0,0,0),
            INTAKE_RAMP = new SupersystemState(0,0,0,0),
            INTAKE_SHELF = new SupersystemState(0,0,0,0),
            INTAKE_FLOOR = new SupersystemState(0,0,0,0),
            PLACE_HIGH_CONE = new SupersystemState(0,0,0,0),
            PLACE_MID_CONE = new SupersystemState(0,0,0,0),
            PLACE_HIGH_CUBE = new SupersystemState(0,0,0,0),
            PLACE_MID_CUBE = new SupersystemState(0,0,0,0),
            PLACE_LOW = new SupersystemState(0,0,0,0);

    enum SupersystemStates{
        RESTING,
        INTAKING_FLOOR,
        INTAKING_SHELF,
        INTAKING_RAMP,
        PLACEING_HIGH_CONE,
        PLACING_MID_CONE,
        PLACING_HIGH_CUBE,
        PLACING_MID_CUBE,
        PLACING_LOW
    }

    enum IntakeStages{
        GO_TO_SETPOINT,
        TARGET,
        GRAB
    }

    enum PlaceStages{
        GO_TO_SETPOINT,
        TARGET,
        PLACE
    }
}
