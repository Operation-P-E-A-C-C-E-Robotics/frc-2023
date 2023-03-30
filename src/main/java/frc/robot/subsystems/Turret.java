// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.safety.DankPids;
import frc.lib.util.ServoMotor;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Constants.SupersystemTolerance;
import frc.robot.DashboardManager;
import static frc.robot.Constants.Turret.*;

public class Turret extends SubsystemBase {
  private final ServoMotor servoController = new ServoMotor(SYSTEM_CONSTANTS, this::setVoltageWithoutStoppingProfile, this::getAngleRadians, this::getAngularVelocityRadians);

  private final WPI_TalonFX turretMaster = new WPI_TalonFX(MOTOR_PORT);
  private final WPI_CANCoder turretEncoder = new WPI_CANCoder(ENCODER_PORT);
  private double setpoint = 0;

  /** Creates a new Turret. */
  public Turret() {
    turretMaster.configFactoryDefault();

    turretEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    turretEncoder.configMagnetOffset(-114);
    turretEncoder.setPositionToAbsolute();
    turretEncoder.configSensorDirection(Constants.Inversions.TURRET_ENCODER);
    turretMaster.setInverted(Constants.Inversions.TURRET);
    turretMaster.configStatorCurrentLimit(CURRENT_LIMIT);

//    turretMaster.setSelectedSensorPosition(Util.rotationsToCounts(Units.degreesToRotations(turretEncoder.getAbsolutePosition()), SYSTEM_CONSTANTS));
    turretMaster.configForwardSoftLimitEnable(false);
    turretMaster.configReverseSoftLimitEnable(false);
    turretMaster.configForwardSoftLimitThreshold(Util.rotationsToCounts(Units.degreesToRotations(MAX_ANGLE_RAD)));
    turretMaster.configReverseSoftLimitThreshold(Util.rotationsToCounts(Units.degreesToRotations(MIN_ANGLE_RAD)));

    DankPids.registerDankTalon(turretMaster);
  }

  /**
   * set the turret speed, Positive Values should be Counter Clock Wise
   */
  public void setPercent(double speed) {
    servoController.disableLoop();
    // if(getAngleRadians() > MAX_ANGLE_RAD && speed < 0) turretMaster.set(0);
    // else if(getAngleRadians() < MIN_ANGLE_RAD && speed > 0) turretMaster.set(0);
    // else turretMaster.set(speed);
    turretMaster.set(speed);
  }

  /**
   * set the turret voltage, Positive Values should be Counter Clock Wise
   */
  public void setVoltage(double volts){
    servoController.disableLoop();
    setVoltageWithoutStoppingProfile(volts);
  }

  /**
   * enable the feedback loop - takes over from the setPercent and setVoltage methods
   */
  public void enableFeedback(){
    servoController.enableLoop();
  }

  /**
   * set the turret angle, Positive Values should be Counter Clock Wise.
   * this will enable the feedback loop, generate a trajectory, and start following it
   * @param angle {@link Rotation2d}
   */
  public void setAngle(Rotation2d angle){
      setpoint = angle.getRadians();
      enableFeedback();
      servoController.goToState(angle.getRadians());
  }

  /**
   * set the voltage without stopping the profile
   * @param volts voltage
   */
  private void setVoltageWithoutStoppingProfile(double volts){
    if(getAngleRadians() > MAX_ANGLE_RAD && volts > 0) turretMaster.set(0);
    else if(getAngleRadians() < MIN_ANGLE_RAD && volts < 0) turretMaster.set(0);
    else turretMaster.setVoltage(volts);
  }


  /**
   * get the current angle of the turret
   * @return {@link Rotation2d}
   */
  public Rotation2d getAngle(){
     return Rotation2d.fromDegrees(turretEncoder.getPosition());
  }

  /**
   * get the current angular velocity of the turret
   * @return {@link Rotation2d}
   */
  public Rotation2d getAngularVelocity(){
     return Rotation2d.fromDegrees(turretEncoder.getVelocity());
  }

  /**
   * get the current angle of the turret in radians
   * @return radians, CCW is positive
   */
  public double getAngleRadians(){
//   var rotation = Util.countsToRotations(turretMaster.getSelectedSensorPosition(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing);
//   return Units.rotationsToRadians(rotation);
    return getAngle().getRadians();
  }

    /**
     * get the current angular velocity of the turret in radians
     * @return radians, CCW is positive
     */
  public double getAngularVelocityRadians(){
      return getAngularVelocity().getRadians();
  }

  /**
   * determine whether the turret is within the tolerance of the setpoint
   * @param tolerance tolerance
   * @param setpoint setpoint
   * @return true if within tolerance
   */
  public boolean withinTolerance(SupersystemTolerance tolerance, double setpoint){
    return Util.epsilonEquals(getAngleRadians(), setpoint, tolerance.turret);
  }

  /**
   * determine whether the turret is within the tolerance -
   * uses the setpoint set by the setAngle method
   * @param tolerance tolerance
   * @return true if within tolerance
   */
  public boolean withinTolerance(SupersystemTolerance tolerance){
    return withinTolerance(tolerance, setpoint);
  }

  //SIMULATION:
  private final TalonFXSimCollection turretMotorSim = turretMaster.getSimCollection();
  private final CANCoderSimCollection turretEncoderSim = turretEncoder.getSimCollection();
  private final LinearSystemSim<N2, N1, N2> turretSim = new LinearSystemSim<>(servoController.getSystem());
  private  double prevsetpt = 0;
  private final boolean PERIODIC_CONTROL_SIMULATION = false;
  @Override
  public void simulationPeriodic() {
    turretMotorSim.setIntegratedSensorRawPosition(
            (int)Util.rotationsToCounts(
                    Units.radiansToRotations(
                            turretSim.getOutput(0)
                    ),
                    2048,
                    SYSTEM_CONSTANTS.gearing
            )
    );
    turretEncoderSim.setRawPosition(
            (int)Util.rotationsToCounts(
                    Units.radiansToRotations(
                            turretSim.getOutput(0)
                    ),
                    SYSTEM_CONSTANTS.cpr,
                    1
            )
    );
    turretMotorSim.setIntegratedSensorVelocity(
            (int)Util.rotationsToCounts(
              Units.radiansToRotations(
                      turretSim.getOutput(1)
              ),
            1,
            SYSTEM_CONSTANTS.gearing
            )
    );
    turretEncoderSim.setVelocity(
            (int)Util.rotationsToCounts(
                    Units.radiansToRotations(
                            turretSim.getOutput(1)/10
                    ),
                    SYSTEM_CONSTANTS.cpr,
                    1
            )
    );
    var setpt = SmartDashboard.getNumber("turret setpoint", 0);
    if(setpt != prevsetpt && PERIODIC_CONTROL_SIMULATION) setAngle(Rotation2d.fromDegrees(setpt));
    prevsetpt = setpt;
    turretSim.setInput(turretMaster.get() * RobotController.getBatteryVoltage());
    turretSim.update(0.02);
    //print the turret angle to smartdashboard:
    SmartDashboard.putNumber("turret angle", getAngle().getDegrees());
//    turretLigament.setAngle(getAngle());
    DashboardManager.getInstance().drawTurretSim(getAngle().getDegrees());
  }

}
