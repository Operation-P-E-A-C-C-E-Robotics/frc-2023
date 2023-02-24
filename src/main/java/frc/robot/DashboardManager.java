// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.*;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class DashboardManager {
    private final Field2d field = new Field2d();
    private static final DashboardManager instance = new DashboardManager();

    private DashboardManager() {}

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

    public void drawWrist(Wrist wrist){
        SmartDashboard.putBoolean("Wrist Flipped", wrist.flipping());
        SmartDashboard.putNumber("Wrist Angle", wrist.getAngle().getDegrees());
    }

    public void drawTurret(Turret turret) {
        SmartDashboard.putNumber("Turret Angle", turret.getAngle().getDegrees());
    }

    public void drawArm(Arm arm) {
        SmartDashboard.putNumber("Arm Extension", arm.getExtension());
        
    }

    public void drawPivot(Pivot pivot) {
        SmartDashboard.putNumber("Arm Rotation", pivot.getRotation().getDegrees());
        SmartDashboard.putBoolean("Brake Enabled", pivot.getBrakeEnabled());
    }

    public void update(){
        SmartDashboard.putData(field);
    }


}
