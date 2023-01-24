// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.sensors.ApriltagLimelight;
import frc.lib.sensors.Limelight;
import frc.lib.sensors.PigeonHelper;
import frc.robot.commands.auto.paths.Paths;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.PeaccyDrive;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //sensors
  private final PigeonHelper pigeon = new PigeonHelper(new PigeonIMU(20));
  private final ApriltagLimelight apriltagLimelight = new ApriltagLimelight();

  //subsystems
  private final Arm arm = new Arm();
  private final Pivot pivot = new Pivot();
  private final DriveTrain driveTrain = new DriveTrain(pigeon);
  private final Turret turret = new Turret();
  private final Wrist wrist = new Wrist();
  private final Supersystem supersystem = new Supersystem(arm, pivot, turret, wrist);


  //OI
  private final Joystick driverJoystick = new Joystick(Constants.OperatorInterface.DRIVER_JOYSTICK);
  private final SendableChooser<Command> teleopDriveMode = new SendableChooser<Command>();

  //commands
  private final PeaccyDrive peaccyDrive = new PeaccyDrive(driveTrain, driverJoystick);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(driveTrain, driverJoystick);
  private final RobotState robotState = new RobotState(driveTrain, supersystem, pigeon, apriltagLimelight);
  private final Paths testPaths = new Paths(robotState, driveTrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    teleopDriveMode.setDefaultOption("Arcade Drive", arcadeDrive);
    teleopDriveMode.addOption("Cheezy Drive",peaccyDrive);
    SmartDashboard.putData("Drive Mode", teleopDriveMode);
  }

  public void update(){
    robotState.update();
    DashboardManager.getInstance().update();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return testPaths.testPath(robotState);
  }
  public Command teleCommand() {
    return teleopDriveMode.getSelected();
  }

  public void setDriveTrainCommand() {
    driveTrain.setDefaultCommand(teleCommand());
  }

}
