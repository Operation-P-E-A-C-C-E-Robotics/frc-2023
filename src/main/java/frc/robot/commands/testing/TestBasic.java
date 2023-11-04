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
    private final Joystick testJoystick;

    /**
     * Test basic functionality of all subsystems
     * (simple set speed commands - no pid, get sensor values)
     * @param arm the arm subsystem
     * @param pivot the pivot subsystem
     * @param turret the turret subsystem
     * @param wrist the wrist subsystem
     */
    public TestBasic(Supersystem supersystem, Arm arm, Pivot pivot, Turret turret, Wrist wrist, Joystick joystick) {
        addRequirements(supersystem, arm, pivot, turret, wrist);
        this.arm = arm;
        this.pivot = pivot;
        this.turret = turret;
        this.wrist = wrist;
        testJoystick = joystick;
    }

    @Override
    public void initialize(){
        System.out.println("Testing basic controls - initialized");
    }

    @Override
    public void execute(){
        // System.out.println("Testing basic controls - executing");
        SmartDashboard.putNumber("arm extension", arm.getExtension());
        SmartDashboard.putNumber("pivot angle (rad)", Units.radiansToDegrees(pivot.getAngleRadians()));
        SmartDashboard.putNumber("turret angle (rad)", turret.getAngle().getRadians());
        SmartDashboard.putNumber("wrist angle (rad)", wrist.getAngle().getRadians());
        SmartDashboard.putNumber("wrist flip angle (rad)", wrist.getWristFlipAngle().getRadians());
        SmartDashboard.putBoolean("wrist flipping", wrist.flipping());
        SmartDashboard.putNumber("arm counts", arm.getEncoderCounts());
        var armSpeed = -Util.handleDeadbandWithSlopeIncrease(testJoystick.getRawAxis(3), 0.1);
        var pivotSpeed = Util.handleDeadbandWithSlopeIncrease(testJoystick.getRawAxis(1), 0.1);
        var turretJoy = testJoystick.getRawAxis(2);
        var turretSpeed = Util.handleDeadbandWithSlopeIncrease(Math.copySign(Math.pow(turretJoy,2), turretJoy), 0.05) * 0.13;
        var wristSpeed = Util.handleDeadbandWithSlopeIncrease(testJoystick.getRawAxis(0), 0.3);
        arm.setPercent(armSpeed);
        pivot.setPercent(pivotSpeed);
        turret.setPercent(turretSpeed);
        wrist.setPercent(wristSpeed);
    }
}
