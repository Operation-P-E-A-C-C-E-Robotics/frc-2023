// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.ButtonMap;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.drive.TestVelocity;
import frc.robot.commands.supersystem.Automations;
import frc.robot.commands.supersystem.DefaultStatemachine;
import frc.robot.commands.supersystem.GoToFieldPoint;
import frc.robot.commands.supersystem.Automations.PlaceLevel;
import frc.robot.commands.supersystem.Setpoints;
import frc.robot.commands.testing.TestBasic;
import frc.robot.commands.testing.TestChickenHead;
import frc.robot.commands.testing.TestPosition;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.sensors.PigeonHelper;
import frc.robot.commands.auto.paths.Paths;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.PeaccyDrive;


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

  private final Compressor compressor = new Compressor(6, PneumaticsModuleType.REVPH);

  //subsystems
  private final DriveTrain driveTrain = new DriveTrain(pigeon);
  private final Turret turret = new Turret();
  private final Pivot pivot = new Pivot(false);
  private final Arm arm = new Arm(pivot::getAngleRadians);
  private final Wrist wrist = new Wrist(pivot::getAngleRadians);
  private final EndEffector endEffector = new EndEffector();
  private final Supersystem supersystem = new Supersystem(arm, pivot, turret, wrist);

  private final RobotState robotState = new RobotState(driveTrain, supersystem, pigeon, apriltagLimelight, armLimelight);
  private final Paths testPaths = new Paths(robotState, driveTrain);
  private final Constraints constraints = new Constraints(supersystem.getKinematics());

  //OI
  private final Joystick driverJoystick = new Joystick(Constants.OperatorInterface.DRIVER_JOYSTICK);
  private final SendableChooser<Command> teleopDriveMode = new SendableChooser<Command>();

  //commands
  private final PeaccyDrive peaccyDrive = new PeaccyDrive(
    driveTrain,
    () -> {return constraints.constrainJoystickFwdJerk(driverJoystick.getY());},
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
  private final Command placeCubeMid = Automations.placeCube(supersystem, endEffector, PlaceLevel.MID, robotState);
  private final Command placeConeMid = Automations.placeConeNoVision(supersystem, endEffector, PlaceLevel.MID, robotState);

  private final ButtonMap.OIEntry[] driverOI = {
          ButtonMap.OIEntry.toggle(placeCubeMid, 3),
          ButtonMap.OIEntry.toggle(placeConeMid, 4),
  };

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    compressor.enableAnalog(60, 80);
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
    new ButtonMap(driverJoystick).map(driverOI);
    // new JoystickButton(driverJoystick, 3).onTrue(new RunCommand(() -> {
    //   var path = testPaths.driveToConeCommand(robotState, driveTrain).get(null);
    //   if(path != null) path.schedule();
    // }, driveTrain));
    supersystem.setDefaultCommand(new DefaultStatemachine(
      supersystem,
      () -> robotXInRange(0, 4.5),
      () -> robotXInRange(12, 30),
      () -> robotState.getOdometryPose().getRotation().getRadians()
   ));
      // supersystem.setDefaultCommand(new TestBasic(supersystem, arm, pivot, turret, wrist));
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
    // An example command will be run in autonomous
    return Autos.testAuto1(testPaths, robotState, supersystem);
  }

  public void setDriveTrainCommand() {
    driveTrain.setDefaultCommand(teleopDriveMode.getSelected());
  }

}
