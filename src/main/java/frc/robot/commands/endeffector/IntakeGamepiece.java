package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffector;

public class IntakeGamepiece extends CommandBase {
    private final EndEffector endEffector;
    private final boolean isIntakingCone;

    /**
     * Intake a gamepiece.
     * @param endEffector the intake subsystem
     * @param isIntakingCone true if we're intaking a cone, false for a cube (controls whether the jaw closes)
     */
    public IntakeGamepiece(EndEffector endEffector, boolean isIntakingCone){
        this.endEffector = endEffector;
        this.isIntakingCone = isIntakingCone;
        addRequirements(endEffector);
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        endEffector.setPercent(1);
        endEffector.setClaw(!(endEffector.colorSensorSeesThing()));
    }

    @Override
    public boolean isFinished(){
        return endEffector.hasCone() || endEffector.hasCube();
    }

    @Override
    public void end(boolean interrupted){
        endEffector.setPercent(0);
        endEffector.setClaw(!isIntakingCone);
    }
}
