package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffector;

public class IntakeDefault extends CommandBase {
    private final EndEffector endEffector;

    public IntakeDefault(EndEffector endEffector){
        this.endEffector = endEffector;
        addRequirements(endEffector);
    }

    @Override
    public void execute(){
        if(endEffector.colorSensorSeesThing() && !(endEffector.hasCube() || endEffector.hasCone())) {
            endEffector.setPercent(0.5);
            endEffector.setClaw(false);
        } else if (endEffector.hasCube() || endEffector.hasCone()){
            endEffector.setPercent(0.02);
        }
    }
}
