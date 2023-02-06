package frc.robot.commands.testing;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Wrist;

public class TestBasic extends CommandBase {
    private final Arm arm;
    private final Pivot pivot;
    private final Turret turret;
    private final Wrist wrist;
    private final Joystick testJoystick = new Joystick(2);

    /**
     * Test basic functionality of all subsystems
     * (simple set speed commands - no pid, get sensor values)
     * @param arm the arm subsystem
     * @param pivot the pivot subsystem
     * @param turret the turret subsystem
     * @param wrist the wrist subsystem
     */
    public TestBasic(Arm arm, Pivot pivot, Turret turret, Wrist wrist) {
        addRequirements(arm, pivot, turret, wrist);
        this.arm = arm;
        this.pivot = pivot;
        this.turret = turret;
        this.wrist = wrist;
    }

    @Override
    public void initialize(){
        System.out.println("Testing basic controls - initialized");
    }

    @Override
    public void execute(){
        System.out.println("Testing basic controls - executing");
        SmartDashboard.putNumber("arm extension", arm.getExtension());
        SmartDashboard.putNumber("pivot angle (deg)", pivot.getAngleRadians());
        SmartDashboard.putNumber("turret angle (deg)", turret.getAngle().getDegrees());
        SmartDashboard.putNumber("wrist angle (deg)", wrist.getAngle().getDegrees());
        SmartDashboard.putBoolean("wrist flipping", wrist.flipping());
        var armSpeed = testJoystick.getRawAxis(1);
        var pivotSpeed = testJoystick.getRawAxis(2);
        var turretSpeed = testJoystick.getRawAxis(3);
        var wristSpeed = testJoystick.getRawAxis(4);
        arm.setPercent(armSpeed);
        pivot.setPercent(pivotSpeed);
        turret.setPercent(turretSpeed);
        wrist.setPercent(wristSpeed);
        SmartDashboard.putNumber("arm speed", armSpeed);
        SmartDashboard.putNumber("pivot speed", pivotSpeed);
        SmartDashboard.putNumber("turret speed", turretSpeed);
        SmartDashboard.putNumber("wrist speed", wristSpeed);
    }
}
