// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Util;

import static frc.robot.Constants.Turret.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
public class Turret extends SubsystemBase {
  private final WPI_TalonFX turretMaster = new WPI_TalonFX(TURRET_MOTOR);


  /** Creates a new Turret. */
  public Turret() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler ru
  }

  /**
   * set the turret speed, Positive Values should be Counter Clock Wise
   */
  public void setPercent(double speed) {
    turretMaster.set(speed);
  }

  /**
   * set the angle of the turret with a {@link Rotation2d}
   * 0 is the front of the robot, positive values
   * turn the turret counterclockwise
   * @param angle {@link Rotation2d}
   */
  public void setAngle(Rotation2d angle){
    //todo
  }

  public void setVelocity(double rotationsPerSecond){
    //todo
  }

  public boolean finishedMotion(){
    return false; //todo
  }

  /**
   * get the current angle of the turret
   * @return {@link Rotation2d}
   */
  public Rotation2d getAngle(){
    var rotation = Util.countsToRotations(turretMaster.getSelectedSensorPosition(), 2048, 0); //todo  Gear Ratiow
    return Rotation2d.fromDegrees(rotation*360);

  }
}
