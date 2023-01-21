// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class DashboardManager {
    private static ShuffleboardTab tab = Shuffleboard.getTab("Competiton");
    private static GenericEntry driveTrainWidget = tab.add("Drivetrain", 0).withWidget(BuiltInWidgets.kDifferentialDrive).getEntry();
    private static DriveTrain driveTrain;

    public DashboardManager(DriveTrain driveTrain) {
        this.driveTrain = driveTrain;
        
    }

    public static void updateDrivetrain(DifferentialDrive differentialDrive) {
        SmartDashboard.putData(differentialDrive);
     }
}
