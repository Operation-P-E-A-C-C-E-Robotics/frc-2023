// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  public Intake() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * spin the intake, 1 should intake and -1 should spit out
   * @param speed
   */
  public void setPercent(double speed) {

    throw new UnsupportedOperationException("Method not implimented");
  }

  /**
   * helper function to intake objects ensuring the wheels are spining the right way
   */
  public void intake() {
    throw new UnsupportedOperationException("Method not implimented");
  }

  /**
   * helper function to eject objects ensuring the wheels are spinning the right way
   */
  public void eject() {
    throw new UnsupportedOperationException("Method not implimented");
  }
  /**
   * open the Pnumatic arms, do not confuse with intake()
   */
  public void open() {
    throw new UnsupportedOperationException("Method not implimented");
  }

  /**
   * helper function to set all settings for intaking a cube
   */
  public void intakeCube() {
    throw new UnsupportedOperationException("Method not implimented");
  }

  /**
   * helper function to set all settings for intaking a cone
   */ 
  public void intakeCone() {
    throw new UnsupportedOperationException("Method not implimented");
  }
}
