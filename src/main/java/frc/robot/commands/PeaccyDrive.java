package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.CheesyDriveHelper;
import frc.robot.subsystems.DriveTrain;

public class PeaccyDrive extends CommandBase {
    private final DriveTrain driveTrain;
    private final CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();
    private final Joystick driveJoystick;

    public PeaccyDrive(DriveTrain driveTrain, Joystick driveJoystick){
        this.driveTrain = driveTrain;
        this.driveJoystick = driveJoystick;

        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double throttle = -driveJoystick.getY();
        double wheel = driveJoystick.getX();
        boolean quickturn = driveJoystick.getRawButton(1);
        driveTrain.tankDrive(cheesyDriveHelper.cheesyDrive(throttle, wheel, quickturn, true));
    }
}
