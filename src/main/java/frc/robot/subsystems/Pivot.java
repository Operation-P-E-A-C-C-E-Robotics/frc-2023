// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Pivot.*;

public class Pivot extends SubsystemBase {
  private WPI_TalonFX pivotMaster = new WPI_TalonFX(PIVOT_MASTER);
  private WPI_TalonFX pivotSlave = new WPI_TalonFX(PIVOT_SLAVE);
  /** Creates a new ExampleSubsystem. */
  public Pivot() {

    pivotSlave.follow(pivotMaster);
    pivotMaster.setNeutralMode(NeutralMode.Brake);

  }

  public void setPercent(double speed){
    pivotMaster.set(speed);
  }

  public void setAnglePID(Rotation2d angle){
    //todo
  }

  public void setAngle(Rotation2d angle){
    //todo
  }

  public Rotation2d getAngle(){
    return null; //todo
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
