// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.safety.DankPids;
import frc.lib.util.ServoArm;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Constants.SupersystemTolerance;
import frc.robot.DashboardManager;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Pivot.*;

public class Pivot extends SubsystemBase {
  private final ServoArm servoController;

  private final WPI_TalonFX pivotMaster = new WPI_TalonFX(PIVOT_MASTER),
          pivotSlave = new WPI_TalonFX(PIVOT_SLAVE);
  private final WPI_CANCoder pivotEncoder = new WPI_CANCoder(PIVOT_ENCODER);
  private final CANCoderFaults faults = new CANCoderFaults();
  private double setpoint = 0;


  /** Creates a new ExampleSubsystem. */
  public Pivot(boolean withBrake, DoubleSupplier armLengthSupplier) {
    servoController = new ServoArm(SYSTEM_CONSTANTS, armLengthSupplier, this::setVoltage, this::getAngleRadians, this::getAngularVelocityRadiansPerSecond, MASS);

    pivotMaster.configFactoryDefault();
    pivotSlave.configFactoryDefault();
    pivotEncoder.configFactoryDefault();

    pivotMaster.setNeutralMode(NeutralMode.Brake);
    pivotSlave.setNeutralMode(NeutralMode.Brake);

    pivotSlave.follow(pivotMaster);

    pivotMaster.setInverted(Constants.Inversions.PIVOT);
    pivotSlave.setInverted(InvertType.OpposeMaster);

    pivotMaster.configStatorCurrentLimit(CURRENT_LIMIT);
    pivotSlave.configStatorCurrentLimit(CURRENT_LIMIT);

    pivotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    pivotEncoder.configSensorDirection(Constants.Inversions.PIVOT_ENCODER);
    pivotEncoder.configMagnetOffset(169);
    pivotEncoder.setPositionToAbsolute();

    pivotMaster.setSelectedSensorPosition(Util.rotationsToCounts(Units.degreesToRotations(pivotEncoder.getAbsolutePosition()), 2048, SYSTEM_CONSTANTS.gearing));
    pivotMaster.configForwardSoftLimitEnable(false);
    pivotMaster.configReverseSoftLimitEnable(false);
    pivotMaster.configForwardSoftLimitThreshold(Util.rotationsToCounts(Units.radiansToRotations(MAX_ANGLE_RAD), 2048, SYSTEM_CONSTANTS.gearing));
    pivotMaster.configReverseSoftLimitThreshold(Util.rotationsToCounts(Units.radiansToRotations(MIN_ANGLE_RAD), 2048, SYSTEM_CONSTANTS.gearing));

    if(Robot.isSimulation()) {
      initializeSimulation();
      SmartDashboard.putNumber("pivot setpoint angle", 0);
      SmartDashboard.putNumber("pivot setpoint percent", 0);
    }

    DankPids.registerDankTalon(pivotMaster);
    DankPids.registerDankTalon(pivotSlave);
  }

  /**
   * set the pivot speed - positive values move towards the front of the robot.
   * @param speed -1 to 1
   */
  public void setPercent(double speed){
    servoController.disableLoop();
    if(getAngleRadians() > MAX_ANGLE_RAD && speed > 0) pivotMaster.set(0);
    else if(getAngleRadians() < MIN_ANGLE_RAD && speed < 0) pivotMaster.set(0);
    else pivotMaster.set(speed);
  }

  private void reconfigureDevicesPeriodic(){
    if(pivotMaster.hasResetOccurred() || pivotSlave.hasResetOccurred()){
      servoController.disableLoop();
      pivotMaster.configFactoryDefault();
      pivotSlave.configFactoryDefault();

      pivotMaster.setNeutralMode(NeutralMode.Brake);
      pivotSlave.setNeutralMode(NeutralMode.Brake);

      pivotSlave.follow(pivotMaster);
      pivotMaster.setNeutralMode(NeutralMode.Brake);

      pivotMaster.setInverted(Constants.Inversions.PIVOT);
      pivotSlave.setInverted(InvertType.OpposeMaster);

      pivotMaster.configStatorCurrentLimit(CURRENT_LIMIT);
      pivotSlave.configStatorCurrentLimit(CURRENT_LIMIT);

      pivotMaster.setSelectedSensorPosition(Util.rotationsToCounts(Units.degreesToRotations(pivotEncoder.getAbsolutePosition()), 2048, SYSTEM_CONSTANTS.gearing));
    }
    pivotEncoder.getFaults(faults);
    if(faults.ResetDuringEn){
      servoController.disableLoop();
      pivotEncoder.configFactoryDefault();

      pivotEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
      pivotEncoder.setPositionToAbsolute();
      pivotEncoder.configSensorDirection(Constants.Inversions.PIVOT_ENCODER);
      pivotEncoder.configMagnetOffset(169);
    }
    if(faults.hasAnyFault()){
      servoController.disableLoop();
    }
  }

  /**
   * set the pivot voltage - positive values move towards the front of the robot.
   * @param volts -12 to 12
   */
  private void setVoltage(double volts){
    if(getAngleRadians() > MAX_ANGLE_RAD && volts > 0) pivotMaster.set(0);
    else if(getAngleRadians() < MIN_ANGLE_RAD && volts < 0) pivotMaster.set(0);
    else pivotMaster.setVoltage(volts);
  }


  /**
   * set the pivot angle - positive values move towards the front of the robot.
   * this enables feedback control.
   * the arm sim calls -90deg straight down - I will change this to 180 deg straight down.
   *    this function should call 180deg straight down. (zero degrees is straight up)
   *    what a mess. "I should have just used radians." (<--AI Wrote this)
   * @param angle a Rotation2d with the setpoint.
   */
  public void setAngle(Rotation2d angle){
    setpoint = angle.getRadians();
    servoController.enableLoop();
    servoController.goToState(angle.getRadians(), 0);
  }

  /**
   * determine whether the pivot is within tolerance of the setpoint.
   * @param tolerance how close to the setpoint is considered "within tolerance"
   * @param setpoint the setpoint to check against
   * @return true if within tolerance, false otherwise
   */
  public boolean withinTolerance(SupersystemTolerance tolerance, double setpoint){
    return Util.epsilonEquals(getAngleRadians(), setpoint, tolerance.pivot);
  }

  /**
   * determine whether the pivot is within tolerance of the setpoint.
   * uses the last position set with setAngle() as the setpoint.
   * @param tolerance how close to the setpoint is considered "within tolerance"
   * @return true if within tolerance, false otherwise
   */
  public boolean withinTolerance(SupersystemTolerance tolerance){
    return withinTolerance(tolerance, setpoint);
  }

  /**
   * get the current angle of the pivot.
   * positive values are towards the front of the robot.
   * @return a Rotation2d with the current angle.
   */
  public Rotation2d getRotation(){
//    return Rotation2d.fromRotations(Util.countsToRotations(pivotMaster.getSelectedSensorPosition(), SYSTEM_CONSTANTS));
     return Rotation2d.fromDegrees(pivotEncoder.getAbsolutePosition());
  }

  /**
   * get the current angle of the pivot.
   * positive values are towards the front of the robot.
   * @return a double with the current angle in radians.
   */
  public double getAngleRadians(){
    return getRotation().getRadians();
  }



  /**
   * get the current angular velocity of the pivot.
   * positive values are towards the front of the robot.
   * @return a double with the current angular velocity in radians per second.
   */
  public double getAngularVelocityRadiansPerSecond(){
//    return Util.countsToRotations(pivotMaster.getSelectedSensorVelocity(), SYSTEM_CONSTANTS) * 10;
     return Units.degreesToRadians(pivotEncoder.getVelocity());
  }

  //SIMULATION: (DON'T TOUCH)
  //NOTE: the pivot sim calls -90deg straight down (where gravity pull to).
  //I have to figure out how to make it call 0deg straight down.
   private TalonFXSimCollection pivotMotorSim;
  private CANCoderSimCollection pivotEncoderSim;
   private static SingleJointedArmSim pivotSim;
   private double prevAngleSetpoint = 0, prevPercentSetpoint = 0;

  public void initializeSimulation(){
    pivotMotorSim = pivotMaster.getSimCollection();
    pivotEncoderSim = pivotEncoder.getSimCollection();
    pivotSim = new SingleJointedArmSim(
            SYSTEM_CONSTANTS.motor,
            SYSTEM_CONSTANTS.gearing,
            SYSTEM_CONSTANTS.inertia,
            LENGTH,
            -100,
            100,
            MASS,
            true
    );
  }

   @Override
   public void simulationPeriodic() {
     //update the simulation
     pivotSim.setInputVoltage(pivotMaster.get() * RobotController.getBatteryVoltage());
     pivotSim.update(0.02);
     SmartDashboard.putNumber("pivot master output", pivotMaster.get());

     //write the simulation data to the motor sim
     //convert angle from sim to encoder tics:
     pivotMotorSim.setIntegratedSensorRawPosition(
             (int) Util.rotationsToCounts(
                     //account for the fact that the sim calls -90deg straight down by adding 270deg
                     Units.radiansToRotations(pivotSim.getAngleRads() + (Math.PI * 1.5)),
                     2048,
                     SYSTEM_CONSTANTS.gearing
             )
     );
     pivotEncoderSim.setRawPosition(
             (int) Util.rotationsToCounts(
                     Units.radiansToRotations(pivotSim.getAngleRads() + (Math.PI * 1.5)),
                     SYSTEM_CONSTANTS.cpr,
                     1
             )
     );
     pivotMotorSim.setIntegratedSensorVelocity(
             (int) Util.rotationsToCounts(
                     Units.radiansToRotations(pivotSim.getVelocityRadPerSec()),
                     2048,
                     SYSTEM_CONSTANTS.gearing
             )
     );
     pivotEncoderSim.setVelocity(
             (int) Util.rotationsToCounts(
                     Units.radiansToRotations(pivotSim.getVelocityRadPerSec()/10),
                     SYSTEM_CONSTANTS.cpr,
                     1
             )
     );

  //   //get new angle setpoint from dashboard
     var angleSetpoint = SmartDashboard.getNumber("pivot setpoint angle", 0);
     boolean PERIODIC_CONTROL_SIMULATION = false;
     if(angleSetpoint != prevAngleSetpoint && PERIODIC_CONTROL_SIMULATION){
       setAngle(Rotation2d.fromDegrees(angleSetpoint));
       prevAngleSetpoint = angleSetpoint;
     }
     //get new percent setpoint from dashboard
     var percentSetpoint = SmartDashboard.getNumber("pivot setpoint percent", 0);
     if(percentSetpoint != prevPercentSetpoint && PERIODIC_CONTROL_SIMULATION){
       setPercent(percentSetpoint);
       prevPercentSetpoint = percentSetpoint;
     }

     //update visualization
     DashboardManager.getInstance().drawPivotSim(Units.radiansToDegrees(pivotSim.getAngleRads()));

     //write information to dashboard:
     SmartDashboard.putNumber("pivot angle degrees", Math.toDegrees(getAngleRadians()));
     SmartDashboard.putNumber("pivot angular velocity", getAngularVelocityRadiansPerSecond());
     SmartDashboard.putNumber("pivot setpoint degrees", Math.toDegrees(setpoint));
   }
}
