// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

import java.util.function.DoubleSupplier;

public class ArcadeDrive extends CommandBase {
  private final DriveTrain driveTrain;
  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier rotationSupplier;

  /** Creates a new TeleoperatedDriverControl. */
  public ArcadeDrive(DriveTrain driveTrain, DoubleSupplier forward, DoubleSupplier rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.forwardSupplier = forward;
    this.rotationSupplier = rotation;
    this.driveTrain = driveTrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.tankDrive(0,0); //stop the drivetrain before drivers take control
  }

  @Override
  public void execute() {
     driveTrain.arcadeDrive(-forwardSupplier.getAsDouble(), rotationSupplier.getAsDouble());
  }
}
