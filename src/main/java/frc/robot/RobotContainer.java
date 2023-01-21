// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.PeaccyDrive;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.paths.Paths;
import frc.robot.commands.drive.TestVelocity;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //subsystems
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain driveTrain;
  private final RobotState robotState;
  //private final Paths paths = new Paths(odometry, driveTrain);
  private final Constraints constrains;
  
  //commands
  private final PeaccyDrive teleoperatedDriverControl;

  //OI
  private Joystick driverJoystick = new Joystick(Constants.OperatorInterface.DRIVER_JOYSTICK); //left this public for easy accesability, we can make it private if you think we should
  private DashboardManager dashboardManager;
  private Command autoCommand;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain = new DriveTrain(this);
    robotState = new RobotState(this, driveTrain, null);
    constrains = new Constraints(driveTrain, new Arm(), new Pivot(), new Turret(), new Wrist(), robotState);
    teleoperatedDriverControl =  new PeaccyDrive(this.driveTrain, driverJoystick);
    dashboardManager = new DashboardManager(driveTrain, robotState);
    autoCommand = new Paths(robotState, driveTrain).testPath(robotState);
    // Configure the button bindings
    configureBindings();

    //default commands
    //driveTrain.setDefaultCommand(teleoperatedDriverControl);
  }

  public Pose2d getStartingPose(){
    return new Pose2d(0,0,new Rotation2d(0));
  }

  public void updateRobotState(){
    robotState.update();
    System.out.println("");
    System.out.println(robotState.getOdometryPose());
    System.out.println("");
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    new JoystickButton(driverJoystick, 4).whileTrue(teleoperatedDriverControl);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoCommand;
  }
}
