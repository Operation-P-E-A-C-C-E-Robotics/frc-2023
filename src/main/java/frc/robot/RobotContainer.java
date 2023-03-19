// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.ButtonMap;
import frc.lib.util.ButtonMap.MultiButton;
import frc.lib.util.ButtonMap.OIEntry;
import frc.lib.util.ButtonMap.SimpleButton;
import frc.lib.util.ButtonMap.SimplePOV;
import frc.lib.util.Util;
import frc.robot.Constants.SupersystemTolerance;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.auto.BangBangBalancer;
import frc.robot.commands.drive.TestVelocity;
import frc.robot.commands.supersystem.Automations;
import frc.robot.commands.supersystem.Automations.PlaceLevel;
import frc.robot.commands.supersystem.Setpoints;
import frc.robot.commands.testing.TestBasic;
import frc.robot.commands.testing.TestChickenHead;
import frc.robot.commands.testing.TestPosition;
import frc.robot.commands.testing.TestSimpleKinematics;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final Compressor compressor = new Compressor(6, PneumaticsModuleType.REVPH);

  //subsystems
  private final DriveTrain driveTrain = new DriveTrain(pigeon);
  private final Turret turret = new Turret();
  private final Arm arm = new Arm();
  private final Pivot pivot = new Pivot(false, arm::getExtension); //this has to be after arm :(
  private final Wrist wrist = new Wrist(pivot::getAngleRadians);
  private final EndEffector endEffector = new EndEffector();
  private final Supersystem supersystem = new Supersystem(arm, pivot, turret, wrist);

  private final RobotState robotState = new RobotState(driveTrain, supersystem, pigeon, drivetrainLimelight, armLimelight);
  private final Constraints constraints = new Constraints(supersystem.getKinematics());

  //OI
  private final Joystick driverJoystick = new Joystick(Constants.OperatorInterface.DRIVER_JOYSTICK),
          operatorJoystick = new Joystick(Constants.OperatorInterface.OPERATOR_JOYSTICK),
          backupJoystick = new Joystick(Constants.OperatorInterface.BACKUP_JOYSTICK);
  private final SendableChooser<Command> teleopDriveMode = new SendableChooser<Command>(),
                autoMode = new SendableChooser<Command>();

  //commands
  private final Paths paths = new Paths(robotState, driveTrain);
  private final Setpoints setpoints = new Setpoints(
          supersystem,
          () -> Util.handleDeadbandWithSlopeIncrease(backupJoystick.getRawAxis(0), 0.2),
          () -> Util.handleDeadbandWithSlopeIncrease(backupJoystick.getRawAxis(1), 0.2),
          () -> Util.handleDeadbandWithSlopeIncrease(backupJoystick.getRawAxis(2), 0.2),
          () -> Util.handleDeadbandWithSlopeIncrease(backupJoystick.getRawAxis(3), 0.2)
  );
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
  private final TestSimpleKinematics kinematicsManual = new TestSimpleKinematics(arm, pivot, turret, wrist, supersystem);

  private final OIEntry[] driverOI = {
    SimpleButton.onHold(new BangBangBalancer(driveTrain, robotState, false), 12),
    SimpleButton.onPress(new InstantCommand(() -> wrist.zero(), wrist), 8),
    SimpleButton.onPress(new InstantCommand(() -> driveTrain.resetEncoders(0,0)), 16)
  };

  private final OIEntry[] mainOperatorOI = {
          MultiButton.toggle(automations.placeConeNoVision(PlaceLevel.HIGH).andThen(new TestSimpleKinematics(arm, pivot, turret, wrist, supersystem)), 4, 7),
          MultiButton.toggle(automations.placeConeNoVision(PlaceLevel.MID).andThen(new TestSimpleKinematics(arm, pivot, turret, wrist, supersystem)), 1, 7),
          MultiButton.toggle(automations.placeConeDriverAssist(PlaceLevel.MID, operatorJoystick::getX, operatorJoystick::getY, operatorJoystick::getZ).andThen(new TestSimpleKinematics(arm, pivot, turret, wrist, supersystem)), 2, 7),
          MultiButton.toggle(automations.placeCube(PlaceLevel.HIGH), 4, 5),
          MultiButton.toggle(automations.placeCube(PlaceLevel.MID), 1, 5),
          MultiButton.toggle(automations.placeCube(PlaceLevel.LOW), 2, 5),
          // SimpleButton.onPress(automations.pickUpConeFloor(), 5),
          // SimpleButton.onPress(automations.pickUpCubeFloor(), 6),
          SimpleButton.onHold(automations.smartZero(), 9),
          SimpleButton.onHold(setpoints.goToSetpoint(Setpoints.zero), 10),
          SimpleButton.onHold(setpoints.goToSetpoint(Setpoints.intakeDoubleSubstation, SupersystemTolerance.INTAKE_SUBSTATION), 3), //mid right button
          SimplePOV.onHold(setpoints.goToPlace(PlaceLevel.HIGH, true), 0), //lower left trigger, top button
          SimplePOV.onHold(setpoints.goToPlace(PlaceLevel.MID, true), 90), //lower left trigger, mid left button
          SimplePOV.onHold(setpoints.goToPlace(PlaceLevel.LOW, false), 180), //mid right button
          SimplePOV.onHold(setpoints.goToSetpoint(Setpoints.intakeDoubleSubstation, SupersystemTolerance.INTAKE_SUBSTATION), 270), //mid right button
          SimpleButton.onPress(new InstantCommand(endEffector::toggleClaw, endEffector), 8), //lower right trigger
  };

  private final OIEntry[] manualOperatorOI = {
          MultiButton.onHold(setpoints.goToPlace(PlaceLevel.HIGH, true), 4), //lower left trigger, top button
          MultiButton.onHold(setpoints.goToPlace(PlaceLevel.MID, true), 1), //lower left trigger, mid left button
          SimpleButton.onHold(setpoints.goToPlace(PlaceLevel.LOW, false), 2), //mid right button
          SimpleButton.onHold(setpoints.goToSetpoint(Setpoints.intakeFloor, SupersystemTolerance.INTAKE_GROUND), 6), //upper right trigger
          SimpleButton.onHold(setpoints.goToSetpoint(Setpoints.intakeDoubleSubstation, SupersystemTolerance.INTAKE_SUBSTATION), 3), //mid right button
          SimpleButton.onHold(new TestSimpleKinematics(arm, pivot, turret, wrist, supersystem), 5),
          SimpleButton.onHold(new TestChickenHead(arm, pivot, turret, wrist, supersystem, robotState), 7),
          SimpleButton.onHold(setpoints.goToSetpoint(Setpoints.zero), 10),
          SimplePOV.onHold(new RunCommand(() -> endEffector.setPercent(-1), endEffector), 180), //pov down
          SimplePOV.onHold(new RunCommand(() -> endEffector.setPercent(1), endEffector), 0), //pov up
          SimpleButton.onPress(new InstantCommand(endEffector::toggleClaw, endEffector), 8), //lower right trigger
  };

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    enableCompressor();

    teleopDriveMode.addOption("Arcade Drive", arcadeDrive);
    teleopDriveMode.addOption("Velocity Drive", velocityDrive);
    teleopDriveMode.addOption("seany drive (test)", testDrive);
    teleopDriveMode.setDefaultOption("Peaccy Drive",peaccyDrive);

    Command autoZeroCommand1 = setpoints.goToSetpoint(Setpoints.zero, SupersystemTolerance.DEFAULT, true);
    Command autoZeroCommand2 = setpoints.goToSetpoint(Setpoints.zero, SupersystemTolerance.DEFAULT, true);

    autoMode.addOption("DO NOTHING", null);
    autoMode.addOption("auto balance",
      new BangBangBalancer(driveTrain, robotState, false)
      .raceWith(setpoints.goToSetpoint(Setpoints.ninetyPivot))
    );
    autoMode.addOption("test auto", new DriveDistance(driveTrain, robotState, -3.4));
    autoMode.addOption("helpplease",
      new InstantCommand(()  -> endEffector.setClaw(true), endEffector)
      .andThen(setpoints.goToSetpoint(Setpoints.placeHighCone, SupersystemTolerance.DEFAULT, true).withTimeout(7),
      new InstantCommand(() -> endEffector.setClaw(false), endEffector),
      new RunCommand(() -> endEffector.setPercent(1), endEffector).withTimeout(1),
      new InstantCommand(() -> endEffector.setPercent(0), endEffector),
      new WaitCommand(0.5),
      autoZeroCommand1.withTimeout(0.5),
      new WaitCommand(0.5),
      new DriveDistance(driveTrain, robotState, -6.5))
    );
    SmartDashboard.putData("Auto Mode", autoMode);
    SmartDashboard.putData("Drive Mode", teleopDriveMode);
    configureBindings();
    wristCoastMode();
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
    new ButtonMap(backupJoystick).map(manualOperatorOI);
  //   supersystem.setDefaultCommand(new DefaultStatemachine(
  //     supersystem,
  //     () -> robotXInRange(0, 4.5),
  //     () -> robotXInRange(12, 30),
  //     () -> robotState.getOdometryPose().getRotation().getRadians()
  //  ));
      supersystem.setDefaultCommand(new TestBasic(supersystem, arm, pivot, turret, wrist, operatorJoystick));
      // pivot.setDefaultCommand(new TestBasic(supersystem, arm, pivot, turret, wrist));
  }

  public void wristBrakeMode(){
    wrist.setNeutralMode(NeutralMode.Brake);
  }

  public void wristCoastMode(){
    wrist.setNeutralMode(NeutralMode.Coast);
  }

  public void enableCompressor(){
    compressor.enableAnalog(Constants.PNEUMATICS_MIN_PRESSURE, Constants.PNEUMATICS_MAX_PRESSURE);
  }

  public void disableCompressor(){
    compressor.disable();
  }

  public void update() {
    robotState.update();
    DashboardManager.getInstance().update();
  } 

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoMode.getSelected();
  }

  public void setDriveTrainCommand() {
    driveTrain.setDefaultCommand(teleopDriveMode.getSelected());
  }

}
