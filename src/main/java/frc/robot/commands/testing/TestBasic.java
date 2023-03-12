package frc.robot.commands.testing;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.PeaccyDriveHelper;
import frc.lib.util.Util;
import frc.robot.subsystems.*;

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
    public TestBasic(Supersystem supersystem, Arm arm, Pivot pivot, Turret turret, Wrist wrist) {
        addRequirements(supersystem, arm, pivot, turret, wrist);
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
        SmartDashboard.putNumber("pivot angle (deg)", Units.radiansToDegrees(pivot.getAngleRadians()));
        SmartDashboard.putNumber("turret angle (deg)", turret.getAngle().getDegrees());
        SmartDashboard.putNumber("wrist angle (deg)", wrist.getAngle().getDegrees());
        SmartDashboard.putBoolean("wrist flipping", wrist.flipping());
        SmartDashboard.putNumber("arm counts", arm.getEncoderCounts());
        var armSpeed = -Util.handleDeadbandWithSlopeIncrease(testJoystick.getRawAxis(3), 0.1);
        var pivotSpeed = Util.handleDeadbandWithSlopeIncrease(testJoystick.getRawAxis(1), 0.1);
        var turretSpeed = PeaccyDriveHelper.applySinCurve(Util.handleDeadbandWithSlopeIncrease(testJoystick.getRawAxis(2), 0.05), false) * 0.2;
        var wristSpeed = Util.handleDeadbandWithSlopeIncrease(testJoystick.getRawAxis(0), 0.3);
        arm.setPercent(armSpeed);
        pivot.setPercent(pivotSpeed);
        turret.setPercent(turretSpeed);
        wrist.setPercent(wristSpeed);
        SmartDashboard.putNumber("arm velocity", arm.getVelocity());
        SmartDashboard.putNumber("arm speed", armSpeed);
        SmartDashboard.putNumber("pivot speed", pivotSpeed);
        SmartDashboard.putNumber("turret speed", turretSpeed);
        SmartDashboard.putNumber("wrist speed", wristSpeed);
    }
}
