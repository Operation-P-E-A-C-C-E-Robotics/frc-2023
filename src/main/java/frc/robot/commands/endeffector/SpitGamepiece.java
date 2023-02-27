package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffector;

public class SpitGamepiece extends CommandBase {
    private final EndEffector endEffector;

    public SpitGamepiece(EndEffector endEffector) {
        this.endEffector = endEffector;
        addRequirements(endEffector);
    }

    @Override
    public void initialize() {
        endEffector.setPercent(-1);
    }

    @Override
    public boolean isFinished() {
        return endEffector.getState() == EndEffector.IntakeState.EJECTING_NOTHING;
    }
}
