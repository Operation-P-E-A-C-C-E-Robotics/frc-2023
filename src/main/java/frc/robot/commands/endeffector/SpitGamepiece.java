package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffector;

import java.util.function.BooleanSupplier;

public class SpitGamepiece extends CommandBase {
    private final EndEffector endEffector;
    private final BooleanSupplier readyToEject;

    public SpitGamepiece(EndEffector endEffector, BooleanSupplier readyToEject) {
        this.endEffector = endEffector;
        this.readyToEject = readyToEject;
        addRequirements(endEffector);
    }

    @Override
    public void execute() {
        if (readyToEject.getAsBoolean()) {
            endEffector.setClaw(true);
            endEffector.setPercent(1);
        } else {
            endEffector.setClaw(false);
            endEffector.setPercent(0);
        }
    }


    @Override
    public boolean isFinished() {
        return endEffector.getState() == EndEffector.IntakeState.EJECTING_NOTHING;
    }
}
