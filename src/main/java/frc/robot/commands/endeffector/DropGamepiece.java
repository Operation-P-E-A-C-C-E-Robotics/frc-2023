package frc.robot.commands.endeffector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffector;

import java.util.function.BooleanSupplier;

public class DropGamepiece extends CommandBase {
    private final EndEffector endEffector;
    private final BooleanSupplier readyToEject;

    public DropGamepiece(EndEffector endEffector, BooleanSupplier readyToEject) {
        this.endEffector = endEffector;
        this.readyToEject = readyToEject;
        addRequirements(endEffector);
    }

    @Override
    public void initialize(){
        endEffector.setClaw(false);
    }

    @Override
    public void execute() {
        endEffector.setClaw(readyToEject.getAsBoolean());
    }

    @Override
    public boolean isFinished(){
        return endEffector.isClawOpen();
    }
}
