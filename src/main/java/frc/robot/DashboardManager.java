// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveTrain;

/** Add your docs here. */
public class DashboardManager {
    // private static ShuffleboardTab tab = Shuffleboard.getTab("Competiton");
    // private static GenericEntry driveTrainWidget = tab.add("Drivetrain", 0).withWidget(BuiltInWidgets.kDifferentialDrive).getEntry();
    private static DriveTrain driveTrain;
    private static RobotState robotState;
    private static Field2d field = new Field2d();

    public DashboardManager(DriveTrain driveTrain, RobotState robotState) {
        this.driveTrain = driveTrain;
        this.robotState = robotState;
        
        
    }

    public static void updateDrivetrain(DifferentialDrive differentialDrive) {
        SmartDashboard.putData("Drivetrain", differentialDrive);
        field.setRobotPose(robotState.getOdometryPose());
        SmartDashboard.putData(field);
        

        
     }
}
