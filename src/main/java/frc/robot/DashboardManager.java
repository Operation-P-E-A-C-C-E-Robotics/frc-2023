// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class DashboardManager {
    private final Field2d field = new Field2d();

    public DashboardManager() {
    }

    public  void updateDriveWidgets(DifferentialDrive differentialDrive, Pose2d robotPose) {
        SmartDashboard.putData("Drivetrain", differentialDrive);
        field.setRobotPose(robotPose.getX(), robotPose.getY(), robotPose.getRotation());
        SmartDashboard.putData(field);
     }
}
