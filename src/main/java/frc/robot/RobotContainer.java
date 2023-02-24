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
import frc.lib.sensors.PigeonHelper;
import frc.robot.commands.auto.paths.Paths;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.PeaccyDrive;
import frc.robot.commands.drive.TestVelocity;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Supersystem;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Wrist;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //sensors
  private final PigeonHelper pigeon = new PigeonHelper(new PigeonIMU(Constants.DriveTrain.PIGEON_IMU));
  private final Limelight apriltagLimelight = new Limelight("limelight"),
                          armLimelight = new Limelight("limelight"); //TODO

  //subsystems
  private final DriveTrain driveTrain = new DriveTrain(pigeon);
  private final Turret turret = new Turret();
  private final Pivot pivot = new Pivot();
  private final Arm arm = new Arm(pivot::getAngleRadians);
  private final Wrist wrist = new Wrist(pivot::getAngleRadians);
  private final Supersystem supersystem = new Supersystem(arm, pivot, turret, wrist);

  //OI
  private final Joystick driverJoystick = new Joystick(Constants.OperatorInterface.DRIVER_JOYSTICK);
  private final SendableChooser<Command> teleopDriveMode = new SendableChooser<Command>();
  private final SendableChooser<Command> autoSelector = new SendableChooser<Command>();

  //commands
  private final PeaccyDrive peaccyDrive = new PeaccyDrive(
    driveTrain,
    driverJoystick::getY,
    driverJoystick::getX,
    () -> driverJoystick.getRawButton(1),
    () -> driverJoystick.getRawButton(2)
  );
  private final TestVelocity velocityDrive = new TestVelocity(driveTrain, driverJoystick, Constants.DriveTrain.DRIVE_KINEMATICS);
  private final ArcadeDrive arcadeDrive = new ArcadeDrive(
    driveTrain,
    driverJoystick::getY,
    driverJoystick::getX,
    () -> driverJoystick.getRawButton(2)
  );

  private final RobotState robotState = new RobotState(driveTrain, supersystem, pigeon, apriltagLimelight, armLimelight);
  private final Paths testPaths = new Paths(robotState, driveTrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureBindings();
    teleopDriveMode.addOption("Arcade Drive", arcadeDrive);
    teleopDriveMode.addOption("Velocity Drive", velocityDrive);
    teleopDriveMode.setDefaultOption("Peaccy Drive",peaccyDrive);
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

  public boolean robotXInRange(double low, double high){
    return robotState.getOdometryPose().getTranslation().getX() > low && robotState.getOdometryPose().getTranslation().getX() < high;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (autoSelector.getSelected() != null) {
      return autoSelector.getSelected();
    } else {
      return null;
    }
  }

  public void setDriveTrainCommand() {
    driveTrain.setDefaultCommand(teleopDriveMode.getSelected());
  }

}
