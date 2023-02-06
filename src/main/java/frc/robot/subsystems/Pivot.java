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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.lib.util.ArmSystemBase;
import frc.lib.util.DCMotorSystemBase;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Constants.SupersystemTolerance;

import static frc.robot.Constants.Pivot.*;

public class Pivot extends ArmSystemBase {
  private final WPI_TalonFX pivotMaster = new WPI_TalonFX(PIVOT_MASTER);
  private final WPI_TalonFX pivotSlave = new WPI_TalonFX(PIVOT_SLAVE);
  private double setpoint = 0;
  private static final double MASS = 1;
  private static final double LENGTH = 0.5;

  /**
   * Creates a new Pivot.
    */
  public Pivot() {
    super(SYSTEM_CONSTANTS, LENGTH, MASS); //pass in constants for the arm controller

    //set up the motor controllers
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
    setpoint = angle.getRadians();
    enableLoop(this::setVoltage, this::getAngleRadians, this::getAngularVelocityRadiansPerSecond);
    goToState(angle.getRadians(), 0);
  }

  public boolean withinTolerance(SupersystemTolerance tolerance, double setpoint){
    return Util.epsilonEquals(getAngleRadians(), setpoint, tolerance.pivot);
  }

  public boolean withinTolerance(SupersystemTolerance tolerance){
    return withinTolerance(tolerance, setpoint);
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
          LENGTH,
          -3,
          3,
          MASS,
          true
  );

  double prevsetpt = 0.0;
  Mechanism2d mech = new Mechanism2d(100, 100);
  MechanismRoot2d root2d = mech.getRoot("pivot", 50, 50);
  MechanismLigament2d pivotLigament = root2d.append(
    new MechanismLigament2d("pivot", 50, 50, 2, new Color8Bit(Color.kRed))
  );

  @Override
  public void simulationPeriodic() {
    SmartDashboard.putData(mech);
    var setpt = SmartDashboard.getNumber("pivot setpoint", 0);
    if(setpt != prevsetpt){
      setPercent(setpt);
    }
    prevsetpt = setpt;
    SmartDashboard.putNumber("isitchanging", setpt);
    //write the motor output to smartdashboard
    SmartDashboard.putNumber("pivot motor output", pivotMaster.get());
    pivotSim.setInput(pivotMaster.get() * RobotController.getBatteryVoltage());
    pivotSim.update(0.02);

    turretMotorSim.setIntegratedSensorRawPosition((int)Util.rotationsToCounts(pivotSim.getAngleRads(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing));
    turretMotorSim.setIntegratedSensorVelocity((int)Util.rotationsToCounts(pivotSim.getVelocityRadPerSec(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing));
    SmartDashboard.putNumber("pivot angle", pivotSim.getAngleRads());
    pivotLigament.setAngle(getRotation());
  }
}
