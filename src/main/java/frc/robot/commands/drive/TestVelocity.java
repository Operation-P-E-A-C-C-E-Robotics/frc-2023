// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class TestVelocity extends CommandBase {
  private DriveTrain driveTrain;
  private Joystick driverJoystick;
  double prevTime;
  DifferentialDriveWheelSpeeds prevSpeeds;
private DifferentialDriveKinematics kinematics;
  /** Creates a new TeleoperatedDriverControl. */
  public TestVelocity(DriveTrain driveTrain, Joystick driverJoystick, DifferentialDriveKinematics kinematics) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driverJoystick = driverJoystick;
    this.driveTrain = driveTrain;
    this.kinematics = kinematics;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevTime = Timer.getFPGATimestamp();
    prevSpeeds = new DifferentialDriveWheelSpeeds(0, 0);
    driveTrain.tankDrive(0,0); //stop the drivetrain before drivers take control
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double time = Timer.getFPGATimestamp();
    double dt = time - prevTime;
    var wheelSpeeds = kinematics.toWheelSpeeds(
        new ChassisSpeeds(driverJoystick.getY() * 0.01, 0, driverJoystick.getX() * 0.01)
    );
    driveTrain.velocityDrive(wheelSpeeds, prevSpeeds, dt);
    prevTime = time;
    prevSpeeds = wheelSpeeds;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
