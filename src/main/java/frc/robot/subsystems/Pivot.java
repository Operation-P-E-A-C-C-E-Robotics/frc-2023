// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.ArmSystemBase;
import frc.lib.util.DCMotorSystemBase;
import frc.lib.util.Util;

import static frc.robot.Constants.Pivot.*;

public class Pivot extends ArmSystemBase {
  private WPI_TalonFX pivotMaster = new WPI_TalonFX(PIVOT_MASTER);
  private WPI_TalonFX pivotSlave = new WPI_TalonFX(PIVOT_SLAVE);

  /** Creates a new ExampleSubsystem. */
  public Pivot() {
    super(SYSTEM_CONSTANTS);

    pivotSlave.follow(pivotMaster);
    pivotMaster.setNeutralMode(NeutralMode.Brake);
    pivotMaster.setInverted(false);
    pivotSlave.setInverted(InvertType.FollowMaster);
    SmartDashboard.putNumber("pivot setpoint", 0);
  }

  public void setPercent(double speed){
    disableLoop();
    pivotMaster.set(speed);
  }

  private void setVoltage(double volts){
    pivotMaster.setVoltage(volts);
  }

  public void setAngle(Rotation2d angle){
    enableLoop(this::setVoltage, this::getAngleRadians, this::getAngularVelocityRadiansPerSecond);
    goToState(angle.getRadians(), 0);
  }

  public boolean finishedMotion(){
    return false; //todo
  }

  public Rotation2d getRotation(){
    var rotation = Units.rotationsToRadians(
            Util.countsToRotations(pivotMaster.getSelectedSensorPosition(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing)
    );
    return new Rotation2d(rotation);
  }

  public double getAngleRadians(){
    return getRotation().getRadians();
  }

  public double getAngularVelocityRadiansPerSecond(){
    return Units.rotationsToRadians(
            Util.countsToRotations(pivotMaster.getSelectedSensorVelocity(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing)
    );
  }

  //simulation
  private final TalonFXSimCollection turretMotorSim = pivotMaster.getSimCollection();
  private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
          SYSTEM_CONSTANTS.motor,
          SYSTEM_CONSTANTS.gearing,
          SYSTEM_CONSTANTS.inertia,
          0.5,
          0,
          3,
          1,
          true
  );

  double prevsetpt = 0.0;

  @Override
  public void simulationPeriodic() {
    var setpt = SmartDashboard.getNumber("pivot setpoint", 0);
    if(setpt != prevsetpt){
      setAngle(new Rotation2d(setpt));
    }
    prevsetpt = setpt;
    SmartDashboard.putNumber("isitchanging", setpt);
    //write the motor output to smartdashboard
    SmartDashboard.putNumber("pivot motor output", pivotMaster.get());
    pivotSim.setInput(pivotMaster.get() * RobotController.getBatteryVoltage());
    pivotSim.update(0.02);
    turretMotorSim.setIntegratedSensorRawPosition((int)Util.rotationsToCounts(pivotSim.getAngleRads()/(2*Math.PI), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing));
    turretMotorSim.setIntegratedSensorVelocity((int)Util.rotationsToCounts(pivotSim.getVelocityRadPerSec()/(2*Math.PI), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing));
    SmartDashboard.putNumber("pivot angle", pivotSim.getAngleRads());
  }
}
