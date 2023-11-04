package frc.robot.commands.testing;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Util;
import frc.robot.Kinematics;
import frc.robot.RobotState;
import frc.robot.subsystems.*;

public class TestSimpleKinematics extends CommandBase {
    private final Supersystem supersystem;
    private final Arm arm;
    private final Pivot pivot;
    private final Turret turret;
    private final Wrist wrist;
    private final Joystick testJoystick = new Joystick(1);
    private double x = 0, y = 0, z = 0;
    private final double SENSITIVITY = 0.01;

    /**
     * Kinematic control of end effector
     * (x, y, z cartesian control of position)
     * @param supersystem the supersystem
     */
    public TestSimpleKinematics(Arm arm, Pivot pivot, Turret turret, Wrist wrist, Supersystem supersystem) {
        addRequirements(supersystem);
        this.supersystem = supersystem;
        this.arm = arm;
        this.pivot = pivot;
        this.turret = turret;
        this.wrist = wrist;
    }

    @Override
    public void initialize(){
        System.out.println("Testing simple kinematics - initialized");
        var pos = supersystem.getKinematics().getSupersystemPosition();
        x = pos.getX();
        y = pos.getY();
        z = pos.getZ();
    }

    @Override
    public void execute(){
        System.out.println("Testing simple kinematics - executing");
        var current = supersystem.getKinematics().getSupersystemPosition();
        SmartDashboard.putNumber("arm extension", arm.getExtension());
        SmartDashboard.putNumber("pivot angle (deg)", pivot.getAngleRadians());
        SmartDashboard.putNumber("turret angle (deg)", turret.getAngle().getDegrees());
        SmartDashboard.putNumber("wrist angle (deg)", wrist.getAngle().getDegrees());
        SmartDashboard.putBoolean("wrist flipping", wrist.flipping());
        SmartDashboard.putNumber("x", current.getX());
        SmartDashboard.putNumber("y", current.getY());
        SmartDashboard.putNumber("z", current.getZ());
        x += Util.handleDeadband(-testJoystick.getRawAxis(3), 0.03) * SENSITIVITY;
        y -= Util.handleDeadband(testJoystick.getRawAxis(2), 0.03) * SENSITIVITY;
        z += Util.handleDeadband(-testJoystick.getRawAxis(1), 0.03) * SENSITIVITY;
        supersystem.setSupersystemPosition(new Translation3d(x, y, z));
        supersystem.setWrist(Rotation2d.fromDegrees(-90));
    }
}
