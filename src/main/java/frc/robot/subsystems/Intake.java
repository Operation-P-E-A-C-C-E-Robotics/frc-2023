// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public static intakeState intakeState = frc.robot.subsystems.Intake.intakeState.OPEN; 
  /** Creates a new Intake. */
  public Intake() {
    switch(intakeState) {
      case OPEN:
        eject(); 
       break;
      case HOLD_CONE:
        intakeCone();
      break;
      case HOLD_CUBE:
        intakeCube();
      break;
      
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * spin the intake, 1 should intake and -1 should spit out
   * @param speed
   */
  public void setPercent(double speed) {

    
  }

  /**
   * helper function to intake objects ensuring the wheels are spining the right way
   */
  public void intake() {
   
  }

  /**
   * helper function to eject objects ensuring the wheels are spinning the right way
   */
  public void eject() {
    
  }
  /**
   * open the Pnumatic arms, do not confuse with intake()
   */
  public void open() {
    
  }

  /**
   * helper function to set all settings for intaking a cube
   */
  public void intakeCube() {
    
  }

  /**
   * helper function to set all settings for intaking a cone
   */ 
  public void intakeCone() {
    
  }

  /**
   * Enum for the intake states
   * OPEN
   * HOLD_CUBE
   * HOLD_CONE
   */
  private enum intakeState {
    OPEN,
    HOLD_CUBE,
    HOLD_CONE

  }

  

}
