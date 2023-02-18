package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.CheesyDriveHelper;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain.Gear;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class PeaccyDrive extends CommandBase {
    private final DriveTrain driveTrain;
    private final CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();
    private final DoubleSupplier throttleSupplier;
    private final DoubleSupplier wheelSpplier;
    private final BooleanSupplier quickturnSupplier;
    private BooleanSupplier gearSupplier;

    public PeaccyDrive(DriveTrain driveTrain, DoubleSupplier throttle, DoubleSupplier wheel, BooleanSupplier quickturn, BooleanSupplier gearSupplier){
        this.driveTrain = driveTrain;
        this.throttleSupplier = throttle;
        this.wheelSpplier = wheel;
        this.quickturnSupplier = quickturn;
        this.gearSupplier = gearSupplier;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double throttle = -throttleSupplier.getAsDouble();
        double wheel = wheelSpplier.getAsDouble();
        boolean quickturn = quickturnSupplier.getAsBoolean();
        boolean isHighGear = !gearSupplier.getAsBoolean();
        driveTrain.tankDrive(cheesyDriveHelper.cheesyDrive(throttle, wheel, quickturn, isHighGear));
        driveTrain.setGear(isHighGear ? Gear.HIGH : Gear.LOW);
    }
}
