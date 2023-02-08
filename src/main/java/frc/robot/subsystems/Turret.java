// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.DCMotorSystemBase;
import frc.lib.util.Util;
import frc.robot.Constants.SupersystemTolerance;
import frc.robot.DashboardManager;

import static frc.robot.Constants.Turret.MOTOR_PORT;
import static frc.robot.Constants.Turret.SYSTEM_CONSTANTS;

public class Turret extends DCMotorSystemBase {
  private final WPI_TalonFX turretMaster = new WPI_TalonFX(MOTOR_PORT);
  private double setpoint = 0;

  /** Creates a new Turret. */
  public Turret() {
    super(SYSTEM_CONSTANTS);
    turretMaster.setInverted(false);
    SmartDashboard.putNumber("turret setpoint", 0);
  }

  /**
   * set the turret speed, Positive Values should be Counter Clock Wise
   */
  public void setPercent(double speed) {
    disableLoop();
    turretMaster.set(speed);
  }

  public void setVoltage(double volts){
    disableLoop();
    setVoltageWithoutStoppingProfile(volts);
  }

  public void enableFeedback(){
    enableLoop(this::setVoltageWithoutStoppingProfile, this::getAngleRaidans, this::getAngularVelocityRaidans);
  }

  public void setAngle(Rotation2d angle){
      setpoint = angle.getRadians();
      enableLoop(this::setVoltageWithoutStoppingProfile, this::getAngleRaidans, this::getAngularVelocityRaidans);
      goToState(angle.getRadians(), 0);
  }
  private void setVoltageWithoutStoppingProfile(double volts){
    turretMaster.setVoltage(volts);
  }


  /**
   * get the current angle of the turret
   * @return {@link Rotation2d}
   */
  public Rotation2d getAngle(){
    var rotation = Util.countsToRotations(turretMaster.getSelectedSensorPosition(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing); //todo  Gear Ratiow
    return Rotation2d.fromDegrees(rotation*360);
  }
  public Rotation2d getAngularVelocity(){
    var velocity = Util.countsToRotations(turretMaster.getSelectedSensorVelocity(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing); //todo  Gear Ratiow
    return Rotation2d.fromDegrees(velocity*360);
  }

  public double getAngleRaidans(){
    return getAngle().getRadians();
  }
  public double getAngularVelocityRaidans(){
      return getAngularVelocity().getRadians();
  }

  public boolean withinTolerance(SupersystemTolerance tolerance, double setpoint){
    return Util.epsilonEquals(getAngleRaidans(), setpoint, tolerance.turret);
  }

  public boolean withinTolerance(SupersystemTolerance tolerance){
    return withinTolerance(tolerance, setpoint);
  }

  //SIMULATION:
  private final TalonFXSimCollection turretMotorSim = turretMaster.getSimCollection();
  private final LinearSystemSim<N2, N1, N2> turretSim = new LinearSystemSim<>(getSystem(), VecBuilder.fill(0.001, 0.001));
  private  double prevsetpt = 0;
  @Override
  public void simulationPeriodic() {
    turretMotorSim.setIntegratedSensorRawPosition(
            (int)Util.rotationsToCounts(
                    Units.radiansToRotations(
                            turretSim.getOutput().get(0,0)
                    ),
                    SYSTEM_CONSTANTS.cpr,
                    SYSTEM_CONSTANTS.gearing
            )
    );
    turretMotorSim.setIntegratedSensorVelocity(
            (int)Util.rotationsToCounts(
              Units.radiansToRotations(
                      turretSim.getOutput().get(1,0)
              ),
            SYSTEM_CONSTANTS.cpr,
            SYSTEM_CONSTANTS.gearing
            )
    );
    var setpt = SmartDashboard.getNumber("turret setpoint", 0);
    if(setpt != prevsetpt) setAngle(Rotation2d.fromDegrees(setpt));
    prevsetpt = setpt;
    turretSim.setInput(turretMaster.get() * RobotController.getBatteryVoltage());
    turretSim.update(0.02);
    //print the turret angle to smartdashboard:
    SmartDashboard.putNumber("turret angle", getAngle().getDegrees());
//    turretLigament.setAngle(getAngle());
    DashboardManager.getInstance().drawTurretSim(getAngle().getDegrees());
  }

}
