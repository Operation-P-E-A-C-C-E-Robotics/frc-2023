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
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constraints;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveTrain;

public class TestVelocity extends CommandBase {
  private final DriveTrain driveTrain;
  private final Joystick driverJoystick;
  private final DifferentialDriveKinematics kinematics;
  private final RobotState robotState;
  private final Constraints constraints;
  private double prevTime;
  private DifferentialDriveWheelSpeeds prevSpeeds;

  private static final double FORWARD_SENSITIVITY = 2,
                              TURN_SENSITIVITY = 1;

  public TestVelocity(DriveTrain driveTrain,
                      Joystick driverJoystick,
                      DifferentialDriveKinematics kinematics,
                      RobotState robotState,
                      Constraints constraints) {
    this.driverJoystick = driverJoystick;
    this.driveTrain = driveTrain;
    this.kinematics = kinematics;
    this.robotState = robotState;
    this.constraints = constraints;
    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    prevTime = Timer.getFPGATimestamp();
    prevSpeeds = new DifferentialDriveWheelSpeeds(0, 0);
  }

  @Override
  public void execute() {
    double time = Timer.getFPGATimestamp();
    double dt = time - prevTime;

    double fwd = constraints.constrainJoystickFwdJerk(-driverJoystick.getY()),
           rot = -driverJoystick.getX();

    var wheelSpeeds = kinematics.toWheelSpeeds(
        new ChassisSpeeds(fwd * TURN_SENSITIVITY, 0, rot * TURN_SENSITIVITY)
    );

    if(driverJoystick.getRawButton(3)){
      driveTrain.resetVelocityDrive();
      robotState.resetOdometry(new Pose2d());
    }

    driveTrain.velocityDrive(wheelSpeeds, prevSpeeds, dt);

    prevTime = time;
    prevSpeeds = wheelSpeeds;
  }
}
