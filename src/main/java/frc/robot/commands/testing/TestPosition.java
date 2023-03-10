package frc.robot.commands.testing;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Util;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Supersystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Wrist;

public class TestPosition extends CommandBase {
    private final Arm arm;
    private final Pivot pivot;
    private final Turret turret;
    private final Wrist wrist;
    private final Joystick testJoystick = new Joystick(2);

    /**
     * Test position control of all subsystems
     * @param arm the arm subsystem
     * @param pivot the pivot subsystem
     * @param turret the turret subsystem
     * @param wrist2 the wrist subsystem
     */
    public TestPosition(Arm arm, Pivot pivot, Turret turret, Wrist wrist2) {
        addRequirements(arm, pivot, turret, wrist2);
        this.arm = arm;
        this.pivot = pivot;
        this.turret = turret;
        this.wrist = wrist2;
    }

    @Override
    public void initialize(){
    }
    double prevWrist = 0, prevTurret = 0, prevPivot = 0, prevArm = 0;

    @Override
    public void execute(){
         System.out.println("Testing position control - executing");
         SmartDashboard.putNumber("arm extension", arm.getExtension());
         SmartDashboard.putNumber("pivot angle (RADEANS)", pivot.getAngleRadians());
         SmartDashboard.putNumber("turret angle (deg)", turret.getAngle().getDegrees());
         SmartDashboard.putNumber("wrist angle (deg)", wrist.getAngle().getDegrees());
         SmartDashboard.putBoolean("wrist flipping", wrist.flipping());
         prevWrist += Util.handleDeadbandWithSlopeIncrease(new Joystick(2).getY(), 0.3)*0.1;
        prevTurret += Util.handleDeadbandWithSlopeIncrease(new Joystick(2).getX(), 0.3)*0.1;
        prevPivot += Util.handleDeadbandWithSlopeIncrease(new Joystick(2).getZ(), 0.3)*0.5;
        prevArm += Util.handleDeadbandWithSlopeIncrease(new Joystick(2).getThrottle(), 0.3)*0.1;
        boolean flipWrist = new Joystick(2).getRawButton(3);
        arm.setExtension(prevArm);
        wrist.setAngle(new Rotation2d(prevWrist));
        pivot.setAngle(new Rotation2d(prevPivot));
        turret.setAngle(new Rotation2d(prevTurret));
        wrist.setFlipped(flipWrist);
        // if(turretSetpoint != prevsetpt){
        //     turret.setAngle(Rotation2d.fromDegrees(turretSetpoint));
        //     prevsetpt = turretSetpoint;
        // }
        // System.out.println(turretSetpoint);
        // wrist.setAngle(Rotation2d.fromDegrees(wristSetpoint));
        // wrist.setFlipped(flipWrist);
    }
}
