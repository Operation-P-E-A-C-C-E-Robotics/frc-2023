// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DashboardManager {
    private final Field2d field = new Field2d();
    private DifferentialDrive differentialDrive;
    private static final DashboardManager instance = new DashboardManager();

    private DashboardManager() {
    }

    public static DashboardManager getInstance(){
        return instance;
    }

    public void drawCone(Pose2d pose){
        field.getObject("cone").setPose(pose);
    }

    public void drawCube(Pose2d pose){
        field.getObject("cube").setPose(pose);
    }

    public void drawTrajectory(Trajectory trajectory){
        field.getObject("trajectory").setTrajectory(trajectory);
    }

    public  void drawDrivetrain(DifferentialDrive differentialDrive, Pose2d robotPose) {
        this.differentialDrive = differentialDrive;
        field.setRobotPose(robotPose.getX(), robotPose.getY(), robotPose.getRotation());
    }

    public void put(){
        SmartDashboard.putData(field);
        SmartDashboard.putData(differentialDrive);
    }
}
