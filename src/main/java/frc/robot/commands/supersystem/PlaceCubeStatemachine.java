package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.Supersystem;

public class PlaceCubeStatemachine extends CommandBase {
    private final Supersystem supersystem;
    private final Translation3d target;
    private final RobotState robotState;
    private State state = State.WAITING_FOR_DRIVETRAIN;
    private final Translation3d targetOffset = new Translation3d(-0.3, 0, 0.3); //TODO

    public PlaceCubeStatemachine(Supersystem supersystem, Translation3d target, RobotState robotState) {
        this.supersystem = supersystem;
        this.target = target.plus(targetOffset);
        this.robotState = robotState;
        addRequirements(supersystem.getRequirements());
    }

    @Override
    public void initialize(){
        state = State.WAITING_FOR_DRIVETRAIN;
    }
    
    @Override
    public void execute(){
        switch(state){
//            case WAITING_FOR_DRIVETRAIN:
//                if(robotState.inRangeOfTarget(new Translation2d(target.getX(), target.getY()))){
//                    state = State.MOVING_ABOVE_TARGET;
//                }
//                break;
//            case MOVING_ABOVE_TARGET:
//                supersystem.set
//                supersystem.setSupersystemPosition(target);
//                if(supersystem.getKinematics().getSupersystemPosition().getZ() < 0.1){
//                    state = State.EJECT;
//                }
//                break;
//            case EJECT:
//                supersystem.setSupersystemPosition(new Translation3d(target.getX(), target.getY(), 0));
//                supersystem.eject();
//                state = State.DONE;
//                break;
//            case DONE:
//                break;
        }
    }

    private enum State{
        WAITING_FOR_DRIVETRAIN,
        MOVING_ABOVE_TARGET,
        EJECT,
        DONE
    }
}
