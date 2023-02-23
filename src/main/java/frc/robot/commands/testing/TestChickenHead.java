package frc.robot.commands.testing;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Kinematics;
import frc.robot.RobotState;
import frc.robot.subsystems.*;

public class TestChickenHead extends CommandBase {
    private final Supersystem supersystem;
    private final Arm arm;
    private final Pivot pivot;
    private final Turret turret;
    private final Wrist wrist;
    private final RobotState robotState;
    private final Joystick testJoystick = new Joystick(2);
    private double x = 0, y = 0, z = 0;
    private final double SENSITIVITY = 0.1;

    /**
     * Test chicken head mode - keep the end effector at a fixed position
     * (x, y, z cartesian control of position relative to the field -
     * test conversions between field and robot coordinates)
     * @param supersystem the supersystem
     */
    public TestChickenHead(Arm arm, Pivot pivot, Turret turret, Wrist wrist, Supersystem supersystem, RobotState robotState) {
        addRequirements(supersystem);
        this.supersystem = supersystem;
        this.arm = arm;
        this.pivot = pivot;
        this.turret = turret;
        this.wrist = wrist;
        this.robotState = robotState;
    }

    @Override
    public void initialize(){
        System.out.println("PREPARING CHICKEN HEAD");
        var posRelativeToEndOfArm = new Pose3d(supersystem.getKinematics().getSupersystemPosition(), new Rotation3d());
        var pos = robotState.drivetrainToField(robotState.turretToDrivetrain(robotState.endEffectorToTurret(posRelativeToEndOfArm)));
        x = pos.getX();
        y = pos.getY();
        z = pos.getZ();
    }

    @Override
    public void execute(){
        System.out.println("CHICKEN HEAD ENGAGED!");
        //get teh current position of the end effector
        var currentEndEffector = new Pose3d(supersystem.getKinematics().getSupersystemPosition(), new Rotation3d());
        var current = robotState.drivetrainToField(robotState.turretToDrivetrain(robotState.endEffectorToTurret(currentEndEffector)));
        SmartDashboard.putNumber("arm extension", arm.getExtension());
        SmartDashboard.putNumber("pivot angle (deg)", pivot.getAngleRadians());
        SmartDashboard.putNumber("turret angle (deg)", turret.getAngle().getDegrees());
        SmartDashboard.putNumber("wrist angle (deg)", wrist.getAngle().getDegrees());
        SmartDashboard.putBoolean("wrist flipping", wrist.flipping());
        SmartDashboard.putNumber("x", current.getX());
        SmartDashboard.putNumber("y", current.getY());
        SmartDashboard.putNumber("z", current.getZ());
        x += testJoystick.getRawAxis(0) * SENSITIVITY;
        y += testJoystick.getRawAxis(1) * SENSITIVITY;
        z += testJoystick.getRawAxis(2) * SENSITIVITY;
        var target = new Pose3d(new Translation3d(x, y, z), new Rotation3d());
        var targetRelativeToEndOfArm = robotState.fieldToDrivetrain(target);
        supersystem.setSupersystemPosition(targetRelativeToEndOfArm.getTranslation());
    }
}
