// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.safety.Inspiration;
import frc.robot.subsystems.PhotonicHRI;
import frc.robot.subsystems.PhotonicHRI.PhotonicLingualElement;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private SendableChooser<PhotonicLingualElement> disabledLEDChooser = new SendableChooser<>();
  private boolean isInMatch;
  private PhotonicLingualElement prev = null;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    robotContainer.wristCoastMode();
    this.isInMatch = Inspiration.initializeInspiration();
    // if(isInMatch) {
    //   Inspiration.inspireDriversInit();
    // } else {
    //   Inspiration.inspireProgrammersInit();
    // }
    disabledLEDChooser.setDefaultOption("rainbow", robotContainer.photonicHRI.rainbow);
    disabledLEDChooser.addOption("off", null);
    disabledLEDChooser.addOption("random", robotContainer.photonicHRI.random);
    disabledLEDChooser.addOption("fire", robotContainer.photonicHRI.fire());
    SmartDashboard.putData("LEDS", disabledLEDChooser);

  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.update();
    // Inspiration.updateSlowPrinter();
  }

  @Override
  public void autonomousInit() {
    robotContainer.wristBrakeMode();
    autonomousCommand = robotContainer.getAutonomousCommand();
    //  Inspiration.inspireAutonomous(isInMatch);

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    robotContainer.wristBrakeMode();
    //  Inspiration.inspireTeleopInit(isInMatch);
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.setDriveTrainCommand();
  }

  @Override
  public void disabledInit(){
    // photonicHRI.runElement(photonicHRI.blink(149, 56, 242, 1));
    //235, 165, 26
    robotContainer.photonicHRI.off();
    robotContainer.wristCoastMode();
  }

  @Override
  public void disabledPeriodic(){
    var selectedLED = disabledLEDChooser.getSelected();
    if(selectedLED == prev) return;
    prev = selectedLED;
    if(selectedLED == null){
      robotContainer.photonicHRI.off();
      return;
    }
    robotContainer.photonicHRI.runElement(selectedLED);
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
