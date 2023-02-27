package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffector;

public class DropGamepiece extends CommandBase {
    private final EndEffector endEffector;

    public DropGamepiece(EndEffector endEffector) {
        this.endEffector = endEffector;
        addRequirements(endEffector);
    }

    @Override
    public void initialize(){
        endEffector.setClaw(true);
    }

    @Override
    public boolean isFinished(){
        return endEffector.isClawOpen();
    }
}
