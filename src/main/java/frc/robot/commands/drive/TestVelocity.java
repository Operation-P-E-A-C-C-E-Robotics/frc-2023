// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constraints;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveTrain;

public class TestVelocity extends CommandBase {
  private final DriveTrain driveTrain;
  private final Joystick driverJoystick;
  double prevTime;
  DifferentialDriveWheelSpeeds prevSpeeds;
private final DifferentialDriveKinematics kinematics;
private RobotState robotState;
  /** Creates a new TeleoperatedDriverControl. */
private Constraints constraints;
  public TestVelocity(DriveTrain driveTrain, Joystick driverJoystick, DifferentialDriveKinematics kinematics, RobotState robotState, Constraints constraints) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driverJoystick = driverJoystick;
    this.driveTrain = driveTrain;
    this.kinematics = kinematics;
    this.robotState = robotState;
    this.constraints = constraints;
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
    double fwd = constraints.constrainJoystickFwdJerk(-driverJoystick.getY());
    var wheelSpeeds = kinematics.toWheelSpeeds(
        new ChassisSpeeds(fwd * 2, 0, -driverJoystick.getX() * 1)
    );
    if(driverJoystick.getRawButton(3)){
      driveTrain.resetVelocityDrive();
      robotState.resetOdometry(new Pose2d());
    }

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
