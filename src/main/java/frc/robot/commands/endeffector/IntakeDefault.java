package frc.robot.commands.intake;

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
        if(endEffector.beamBroken() && !(endEffector.hasCube() || endEffector.hasCone())) {
            endEffector.setPercent(0.3);
        }
        endEffector.setClaw(!endEffector.hasCone());
    }
}
