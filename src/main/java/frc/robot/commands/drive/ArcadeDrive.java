// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Util;
import frc.robot.subsystems.DriveTrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ArcadeDrive extends CommandBase {
  private final DriveTrain driveTrain;
  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier rotationSupplier;
  private BooleanSupplier gearSupplier;

  /** Creates a new TeleoperatedDriverControl. */
  public ArcadeDrive(DriveTrain driveTrain, DoubleSupplier forward, DoubleSupplier rotation, BooleanSupplier gearSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.forwardSupplier = forward;
    this.rotationSupplier = rotation;
    this.driveTrain = driveTrain;
    this.gearSupplier = gearSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.tankDrive(0,0); //stop the drivetrain before drivers take control
  }

  @Override
  public void execute() {
    var gear = gearSupplier.getAsBoolean() ? DriveTrain.Gear.LOW : DriveTrain.Gear.HIGH;
    var forward = Util.handleDeadband(-forwardSupplier.getAsDouble(), 0.04);
    var rotation = Util.handleDeadband(-rotationSupplier.getAsDouble(), 0.04);
    driveTrain.arcadeDrive(-forward, rotation);
    driveTrain.setGear(gear);
  }
}
