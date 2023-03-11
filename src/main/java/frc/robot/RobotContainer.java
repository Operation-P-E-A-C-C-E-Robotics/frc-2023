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
import frc.robot.Constants.SupersystemTolerance;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.BangBangBalancer;
import frc.robot.commands.drive.TestVelocity;
import frc.robot.commands.supersystem.Automations;
import frc.robot.commands.supersystem.Automations.PlaceLevel;
import frc.robot.commands.supersystem.Setpoints;
import frc.robot.commands.testing.TestBasic;
import frc.robot.commands.testing.TestPosition;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.sensors.PigeonHelper;
import frc.robot.commands.auto.paths.Paths;
import frc.robot.commands.drive.ArcadeDrive;
import frc.robot.commands.drive.ChesyDriv;
import frc.robot.commands.drive.DriveDistance;
import frc.robot.commands.drive.SeanyDrive;


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
  private final Limelight drivetrainLimelight = new Limelight("limelight"),
                          armLimelight = new Limelight("armLimelight");

  //subsystems
  private final DriveTrain driveTrain = new DriveTrain(pigeon);
  private final Turret turret = new Turret();
  private final Pivot pivot = new Pivot(false);
  private final Arm arm = new Arm(pivot::getAngleRadians);
  private final Wrist wrist = new Wrist(pivot::getAngleRadians);
  private final EndEffector endEffector = new EndEffector();
  private final Supersystem supersystem = new Supersystem(arm, pivot, turret, wrist);

  private final RobotState robotState = new RobotState(driveTrain, supersystem, pigeon, drivetrainLimelight, armLimelight);
  private final Constraints constraints = new Constraints(supersystem.getKinematics());

  //OI
  private final Joystick driverJoystick = new Joystick(Constants.OperatorInterface.DRIVER_JOYSTICK);
  private final Joystick operatorJoystick = new Joystick(Constants.OperatorInterface.OPERATOR_JOYSTICK);
  private final SendableChooser<Command> teleopDriveMode = new SendableChooser<Command>();

  //commands
  private final Paths paths = new Paths(robotState, driveTrain);
  private final Setpoints setpoints = new Setpoints(supersystem, operatorJoystick::getX, operatorJoystick::getY, operatorJoystick::getZ, operatorJoystick::getThrottle);
  private final Automations automations = new Automations(supersystem, robotState, endEffector, setpoints);
  private final SeanyDrive testDrive = new SeanyDrive(driverJoystick, driveTrain, robotState);
  private final ChesyDriv peaccyDrive = new ChesyDriv(
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

  private final OIEntry[] driverOI = {
    SimpleButton.onHold(new BangBangBalancer(driveTrain, robotState, false), 12)
  };

  private final OIEntry[] mainOperatorOI = {
          SimpleButton.toggle(automations.placeConeNoVision(PlaceLevel.HIGH), 4),
          SimpleButton.toggle(automations.placeConeNoVision(PlaceLevel.MID), 1),
          SimpleButton.toggle(automations.placeConeNoVision(PlaceLevel.LOW), 2),
          SimpleButton.onPress(automations.pickUpConeFloor(), 5),
          SimpleButton.onPress(automations.pickUpCubeFloor(), 6),
  };

  private final OIEntry[] manualOperatorOI = {
          SimpleButton.onHold(new RunCommand(() -> endEffector.setPercent(-1), endEffector), 5),
          SimpleButton.onHold(new RunCommand(() -> endEffector.setPercent(1), endEffector), 6),
          SimpleButton.onPress(new RunCommand(() -> endEffector.setClaw(true), endEffector), 7),
          SimpleButton.onPress(new RunCommand(() -> endEffector.setClaw(false), endEffector), 8),
          SimpleButton.onHold(setpoints.goToSetpoint(Setpoints.placeHighCone, SupersystemTolerance.PLACE_HIGH), 4),
          SimpleButton.onHold(setpoints.goToSetpoint(Setpoints.placeMidCone, SupersystemTolerance.PLACE_MID), 1),
          SimpleButton.onHold(setpoints.goToSetpoint(Setpoints.placeLow, SupersystemTolerance.PLACE_LOW), 2)
  };

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    Compressor compressor = new Compressor(6, PneumaticsModuleType.REVPH);
    compressor.enableAnalog(100, 120);

    teleopDriveMode.addOption("Arcade Drive", arcadeDrive);
    teleopDriveMode.addOption("Velocity Drive", velocityDrive);
    teleopDriveMode.addOption("seany drive (test)", testDrive);
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
    endEffector.setDefaultCommand(new RunCommand(() -> endEffector.setPercent(0), endEffector));
    new ButtonMap(driverJoystick).map(driverOI);
    new ButtonMap(operatorJoystick).map(mainOperatorOI);
    new ButtonMap(new Joystick(2)).map(manualOperatorOI);
    pivot.setDefaultCommand(new TestPosition(arm, pivot, turret, wrist));
  //   supersystem.setDefaultCommand(new DefaultStatemachine(
  //     supersystem,
  //     () -> robotXInRange(0, 4.5),
  //     () -> robotXInRange(12, 30),
  //     () -> robotState.getOdometryPose().getRotation().getRadians()
  //  ));
      // supersystem.setDefaultCommand(new TestBasic(supersystem, arm, pivot, turret, wrist));
      // pivot.setDefaultCommand(new TestBasic(arm, pivot, turret, wrist));
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
