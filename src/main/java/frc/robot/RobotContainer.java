// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.lib.util.ButtonMap;
import frc.lib.util.ButtonMap.OIEntry;
import frc.lib.util.ButtonMap.SimpleButton;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.drive.TestVelocity;
import frc.robot.commands.supersystem.Automations;
import frc.robot.commands.supersystem.DefaultStatemachine;
import frc.robot.commands.supersystem.Automations.PlaceLevel;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
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
  private final Constraints constraints = new Constraints(supersystem.getKinematics());

  //OI
  private final Joystick driverJoystick = new Joystick(Constants.OperatorInterface.DRIVER_JOYSTICK);
  private final Joystick operatorJoystick = new Joystick(Constants.OperatorInterface.OPERATOR_JOYSTICK);
  private final SendableChooser<Command> teleopDriveMode = new SendableChooser<Command>();

  //commands
  private final Paths paths = new Paths(robotState, driveTrain);
  private final Automations automations = new Automations(supersystem, robotState, endEffector);
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

  private final OIEntry[] testDriverOI = {
          SimpleButton.toggle(automations.placeCube(PlaceLevel.HIGH), 3),
          SimpleButton.toggle(automations.placeConeNoVision(PlaceLevel.MID), 4),
  };

  private final OIEntry[] mainOperatorOI = {
          SimpleButton.toggle(automations.place(PlaceLevel.HIGH), 4),
          SimpleButton.toggle(automations.place(PlaceLevel.MID), 1),
          SimpleButton.toggle(automations.place(PlaceLevel.LOW), 2),
  };

  private final OIEntry[] manualOperatorOI = {
          SimpleButton.onHold(new RunCommand(() -> endEffector.setPercent(-1), endEffector), 7),
          SimpleButton.onHold(new RunCommand(() -> endEffector.setPercent(1), endEffector), 8),
  };

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    compressor.enableAnalog(60, 80); //TODO up pressure

    teleopDriveMode.addOption("Arcade Drive", arcadeDrive);
    teleopDriveMode.addOption("Velocity Drive", velocityDrive);
    teleopDriveMode.setDefaultOption("Peaccy Drive",peaccyDrive);
    SmartDashboard.putData("Drive Mode", teleopDriveMode);
    configureBindings();
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
    new ButtonMap(driverJoystick).map(testDriverOI);
    new ButtonMap(operatorJoystick).map(mainOperatorOI);
    new ButtonMap(operatorJoystick).map(manualOperatorOI);
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
    return Autos.testAuto1(paths, robotState, supersystem);
  }

  public void setDriveTrainCommand() {
    driveTrain.setDefaultCommand(teleopDriveMode.getSelected());
  }

}
