// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Add your docs here. */
public class DashboardManager {
    private final Field2d field = new Field2d();
    private static final DashboardManager instance = new DashboardManager();

    private DashboardManager() {
        // this.robotState = robotState;
    }

    public static DashboardManager getInstance(){
        return instance;
    }

    public void drawCone(Pose2d pose){
        field.getObject("cone").setPose(pose);
    }

    public void drawAprilTag(Pose2d pose){
        field.getObject("apriltag").setPose(pose);
    }

    public void drawCube(Pose2d pose){
        field.getObject("cube").setPose(pose);
    }

    public void drawTrajectory(Trajectory trajectory){
        field.getObject("trajectory").setTrajectory(trajectory);
    }

    public  void drawDrivetrain(DifferentialDrive differentialDrive, Pose2d robotPose) {
        // this.differentialDrive = differentialDrive;
        field.setRobotPose(robotPose.getX(), robotPose.getY(), robotPose.getRotation());
    }

    public void update(){
        SmartDashboard.putData(field);
        if(Robot.isSimulation()){
            SmartDashboard.putData("turret", turretMechanism);
            SmartDashboard.putData("arm", armMechanism);
        }
    }

    //simulation visualization:
    Mechanism2d turretMechanism = new Mechanism2d(200, 200);
    MechanismLigament2d turret = turretMechanism.getRoot("Turret", 100, 100).append(
            new MechanismLigament2d("Turret", 90, 0, 3, new Color8Bit(Color.kRed))
    );
    Mechanism2d armMechanism = new Mechanism2d(500, 500);
    MechanismRoot2d armRoot = armMechanism.getRoot("Arm", 250, 100);
    MechanismLigament2d pivot = armRoot.append(
            new MechanismLigament2d("Arm", 150, 0, 3, new Color8Bit(Color.kRed))
    );
    MechanismLigament2d wrist = pivot.append(
            new MechanismLigament2d("Wrist", 50, 0, 3, new Color8Bit(Color.kBlue))
    );

    public void drawTurretSim(double degrees){
        turret.setAngle(degrees);
    }

    public void drawArmSim(double length){
        pivot.setLength(length);
    }

    public void drawPivotSim(double degrees){
        pivot.setAngle(degrees);
    }

    public void drawWristSim(double degrees){
        wrist.setAngle(degrees);
    }
}
