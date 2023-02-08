// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import edu.wpi.first.cameraserver.CameraServer;
// import edu.wpi.first.cscore.UsbCamera;
// import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.safety.Inspiration;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;
  private boolean isInMatch;

  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    this.isInMatch = Inspiration.initializeInspiration();
    if(isInMatch) {
      Inspiration.inspireDriversInit();
    } else {
      Inspiration.inspireProgrammersInit();
    }
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    robotContainer.update();
    Inspiration.updateSlowPrinter();
  }

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    Inspiration.inspireAutonomous(isInMatch);

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void teleopInit() {
    Inspiration.inspireTeleopInit(isInMatch);

    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    robotContainer.setDriveTrainCommand();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
