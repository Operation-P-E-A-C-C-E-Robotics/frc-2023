// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.supersystem.DefaultStatemachine;

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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.util.AllianceFlipUtil;
import frc.lib.util.ButtonMap;
import frc.lib.util.ButtonMap.MultiButton;
import frc.lib.util.ButtonMap.OIEntry;
import frc.lib.util.ButtonMap.Button;
import frc.lib.util.Util;
import frc.robot.Constants.SupersystemTolerance;
import frc.robot.commands.auto.BangBangBalancer;
import frc.robot.commands.auto.MobilityOverStation;
import frc.robot.commands.drive.TestVelocity;
import frc.robot.commands.endeffector.IntakeDefault;
import frc.robot.commands.supersystem.Automations;
import frc.robot.commands.supersystem.Automations.PlaceLevel;
import frc.robot.commands.supersystem.Setpoints;
import frc.robot.commands.testing.TestBasic;
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
  public static PhotonicHRI photonicHRI = new PhotonicHRI(0, 120);

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
  private final SeanyDrive seansFancyDriveMode = new SeanyDrive(driverJoystick, driveTrain, robotState);
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
  private final Command placeConeHigh = automations.placeConeNoVision(PlaceLevel.HIGH),
          placeConeMid = automations.placeConeNoVision(PlaceLevel.MID),
          placeLow = automations.placeCube(PlaceLevel.LOW),
          placeCubeHigh = automations.placeCube(PlaceLevel.HIGH),
          placeCubeMid = automations.placeCube(PlaceLevel.MID);

  private final Command highSetpoint = setpoints.goToPlace(PlaceLevel.HIGH, true),
          midSetpoint = setpoints.goToPlace(PlaceLevel.MID, true),
          lowSetpoint = setpoints.goToPlace(PlaceLevel.LOW, false);

  private final Command intakeFloorFancy = setpoints.groundIntake(operatorJoystick::getPOV, () -> AllianceFlipUtil.apply(robotState.getOdometryPose()).getRotation().getRadians()),
          intakeFloorSimple = setpoints.goToSetpoint(Setpoints.intakeFloor),
          intakeSubstation = setpoints.goToSetpoint(Setpoints.intakeDoubleSubstation);

  private final Command hybridDropOuttake = endEffector.drop().withTimeout(0.1).andThen(endEffector.runOuttake());

  private final OIEntry[] driverOI = {
          Button.onPress(new InstantCommand(wrist::zero, wrist), 8),
          Button.onPress(new InstantCommand(() -> driveTrain.resetEncoders(0,0)), 16)
  };

  private final OIEntry[] autoPlaceBindings = {
          MultiButton.toggle(placeConeHigh, 4, 7),
          MultiButton.toggle(placeConeMid, 1, 7),
          MultiButton.toggle(placeLow, 2, 7),
          MultiButton.toggle(placeCubeHigh, 4, 5),
          MultiButton.toggle(placeCubeMid, 1, 5),
          MultiButton.toggle(placeLow, 2, 5)
  };

  private final OIEntry[] simplifiedAutoPlace = {
        Button.toggle(automations.place(PlaceLevel.HIGH), 4),
        Button.toggle(automations.place(PlaceLevel.MID), 1),
        Button.toggle(automations.place(PlaceLevel.LOW), 2)
  };

  private final OIEntry[] setpointBindings = {
          ButtonMap.SimplePOV.onHold(highSetpoint, 0),
          ButtonMap.SimplePOV.onHold(midSetpoint, 90),
          ButtonMap.SimplePOV.onHold(lowSetpoint, 180),

          Button.onHold(automations.smartZero(), 10),
          Button.onHold(setpoints.goToSetpoint(Setpoints.zero), 9),
  };

  private final OIEntry[] manualBindings = {
          Button.toggle(intakeFloorSimple.alongWith(endEffector.runIntake()), 1),
          Button.toggle(intakeSubstation.alongWith(
                endEffector.runIntake()
                // new RunCommand(() -> wrist.setAngle(Rotation2d.fromRadians(Setpoints.intakeDoubleSubstation.getWristAngle())), wrist)
        ).until(endEffector::colorSensorSeesThing), 3),

          Button.onHold(endEffector.runIntake().until(endEffector::colorSensorSeesThing), 8),
          Button.onHold(endEffector.runOuttake(), 6),
          Button.onHold(endEffector.runIntakeClosed(), 13),
          Button.onPress(new InstantCommand(() -> wrist.zero()),14),
          ButtonMap.JoystickTrigger.onMove(operatorJoystick, new TestBasic(supersystem, arm, pivot, turret, wrist, operatorJoystick), 0, 0.1),
          ButtonMap.JoystickTrigger.onMove(operatorJoystick, new TestBasic(supersystem, arm, pivot, turret, wrist, operatorJoystick), 1, 0.1),
          ButtonMap.JoystickTrigger.onMove(operatorJoystick, new TestBasic(supersystem, arm, pivot, turret, wrist, operatorJoystick), 2, 0.1),
          ButtonMap.JoystickTrigger.onMove(operatorJoystick, new TestBasic(supersystem, arm, pivot, turret, wrist, operatorJoystick), 3, 0.1)
  };

  private final OIEntry[] colorBindings = {
        Button.onPress(new InstantCommand(() -> photonicHRI.runElement(photonicHRI.setSolidColor(255, 255, 0))), 7),
        Button.onPress(new InstantCommand(() -> photonicHRI.runElement(photonicHRI.setSolidColor(255, 0, 255))), 5)
  };

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    enableCompressor();
    arm.setSoftLimit(() -> {
        var currentState = supersystem.getSupersystemState();
        return Constraints.armTooFar(currentState);
    });
    driveTrain.setRobotState(robotState);

    photonicHRI.off();

    teleopDriveMode.addOption("Arcade Drive", arcadeDrive);
    teleopDriveMode.addOption("Velocity Drive", velocityDrive);
    teleopDriveMode.setDefaultOption("seany drive (test)", seansFancyDriveMode);
    teleopDriveMode.addOption("Peaccy Drive",peaccyDrive);

    Command autoZeroCommand1 = setpoints.goToSetpoint(Setpoints.zero, SupersystemTolerance.DEFAULT, true);

    autoMode.addOption("DO NOTHING", null);
    autoMode.addOption("place and balance",
        new InstantCommand(()  -> endEffector.setClaw(true), endEffector)
                    .andThen(setpoints.goToSetpoint(Setpoints.placeHighCone, SupersystemTolerance.DEFAULT, true).withTimeout(2),
                            new InstantCommand(() -> endEffector.setClaw(true), endEffector),
                            new RunCommand(() -> endEffector.setPercent(1), endEffector).withTimeout(1),
                            new InstantCommand(() -> endEffector.setPercent(0), endEffector),
                            new WaitCommand(0.3),
                            setpoints.goToSetpoint(Setpoints.ninetyPivot).withTimeout(0.15),
                            new WaitCommand(0.4),
                            new BangBangBalancer(driveTrain, robotState, false)
    ));
    autoMode.addOption("auto balance",
            new BangBangBalancer(driveTrain, robotState, false)
                    .raceWith(setpoints.goToSetpoint(Setpoints.ninetyPivot))
    );
    autoMode.addOption("test drive distance", new DriveDistance(driveTrain, robotState, -3.4));
    autoMode.addOption("Place and drive",
            new InstantCommand(()  -> endEffector.setClaw(true), endEffector)
                    .andThen(setpoints.goToSetpoint(Setpoints.placeHighCone, SupersystemTolerance.DEFAULT, true).withTimeout(7),
                            new InstantCommand(() -> endEffector.setClaw(false), endEffector),
                            new RunCommand(() -> endEffector.setPercent(1), endEffector).withTimeout(1),
                            new InstantCommand(() -> endEffector.setPercent(0), endEffector),
                            new WaitCommand(0.5),
                            autoZeroCommand1.withTimeout(0.5),
                            new WaitCommand(0.5),
                            new DriveDistance(driveTrain, robotState, 2.5))
    );
    autoMode.addOption("DO NOT RUN THIS (unless you know what it is)", new MobilityOverStation(driveTrain, robotState));
    autoMode.addOption("test path", paths.testPath(robotState));
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
    endEffector.setDefaultCommand(endEffector.rest());
    new ButtonMap(driverJoystick).map(driverOI);
    var operatorMap = new ButtonMap(operatorJoystick);
    operatorMap.map(manualBindings);
    operatorMap.map(simplifiedAutoPlace);
    operatorMap.map(setpointBindings);
    operatorMap.map(autoPlaceBindings);
    operatorMap.map(colorBindings);
//        supersystem.setDefaultCommand(new DefaultStatemachine(
//          supersystem,
//          () -> false,//robotXInRange(0, 4.5),
//          () -> false,//robotXInRange(12, 30),
//          () -> robotState.getOdometryPose().getRotation().getRadians()
//       ));
supersystem.setDefaultCommand(automations.smartZero());
//    supersystem.setDefaultCommand(new TestBasic(supersystem, arm, pivot, turret, wrist, operatorJoystick));
    // pivot.setDefaultCommand(new TestBasic(supersystem, arm, pivot, turret, wrist));

    new Trigger(endEffector::colorSensorSeesThing).onTrue(new InstantCommand(
            () -> photonicHRI.runElement(photonicHRI.setSolidColor(0,255,0))
    ));
  }

  public Supersystem getSupersystem(){
        return supersystem;
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
