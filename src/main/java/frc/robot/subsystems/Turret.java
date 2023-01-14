// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  public Turret() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * set the turret speed, Positive Values should be Counter Clock Wise
   * @param speed
   */
  public void setTurretPercent(double speed) {
  }

  /**
   * set the angle of the turret with a {@link Rotation2d}
   * 0 is the front of the robot, positive values
   * turn the turret counter clockwise
   * @param angle {@link Rotation2d}
   */
  public void setAngle(Rotation2d angle){
    //todo
  }

  public void setVelocity(double rotationsPerSecond){
    //todo
  }

  /**
   * get the current angle of the turret
   * @return {@link Rotation2d}
   */
  public Rotation2d getAngle(){
    return null; //todo
  }
}
