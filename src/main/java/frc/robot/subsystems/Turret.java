// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Turret.*;

public class Turret extends SubsystemBase {
  private WPI_TalonFX turret = new WPI_TalonFX(TURRET);
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
      turret.set(speed);
  }

}
