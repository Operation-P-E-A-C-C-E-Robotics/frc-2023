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
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.ArmSystemBase;
import frc.lib.util.DCMotorSystemBase;
import frc.lib.util.Util;
import frc.robot.Constants.SupersystemTolerance;
import frc.robot.DashboardManager;
import frc.robot.Robot;

import static frc.robot.Constants.Pivot.*;

public class Pivot extends ArmSystemBase {
  private final WPI_TalonFX pivotMaster = new WPI_TalonFX(PIVOT_MASTER);
  private final WPI_TalonFX pivotSlave = new WPI_TalonFX(PIVOT_SLAVE);
  private final DoubleSolenoid brakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, BRAKE_SOLENOID_FORWARD, BRAKE_SOLENOID_BACKWARD);
  private final Timer brakeTimer = new Timer();
  private static boolean brakeState = true;
  private double setpoint = 0;


  /** Creates a new ExampleSubsystem. */
  public Pivot() {
    super(SYSTEM_CONSTANTS, LENGTH, MASS);

    pivotSlave.follow(pivotMaster);
    pivotMaster.setNeutralMode(NeutralMode.Brake);

    pivotMaster.setInverted(false);
    pivotSlave.setInverted(InvertType.FollowMaster);

    setBrakeEnabled(true);
    // brakeSolenoid.initSendable(null);

    if(Robot.isSimulation() && PERIODIC_CONTROL_SIMULATION) {
      SmartDashboard.putNumber("pivot setpoint angle", 0);
      SmartDashboard.putNumber("pivot setpoint percent", 0);
    }

//    addFeedforward((double pos, double vel) -> {
//      var force = (MASS * 9.80665) * LENGTH * Math.cos(pos - Math.PI*0.5);
//      //account for gearing:
//      force /= SYSTEM_CONSTANTS.gearing;
//      SmartDashboard.putNumber("pivot force", force);
//      //calculate voltage needed to counteract force:
//      return (SYSTEM_CONSTANTS.motor.getVoltage(force, vel) * 12.0);
//    });
  }

  /**
   * set the pivot speed - positive values move towards the front of the robot.
   * @param speed -1 to 1
   */
  public void setPercent(double speed){
    disableLoop();
    pivotMaster.set(speed);
  }

  public void setBrakeEnabled(boolean state) {
      brakeTimer.start();
      if (state = true){
        brakeState = true; //Notify the bool that we are engaging the break so nothing else can move the system
        brakeSolenoid.set(Value.kForward); //Forward = Break On
      } else if (state=false){
        brakeSolenoid.set(Value.kReverse); //Reverse = Break Off
        if(brakeTimer.hasElapsed(TIME_FOR_BRAKE_TO_ENGAGE)) { //wait for the break to release before allowing other methods to control the motor
          brakeState = false;  //Notify the bool that we are disengaging the break so everything else can move the system
          brakeTimer.stop();
          brakeTimer.reset();
        }
      } else {
        brakeState = true; 
        brakeSolenoid.set(Value.kReverse); //If we recieve an invalid state, put the brake on just in case, this thing is dangerous
      }
      DashboardManager.getInstance().drawPivotBrakeBool(brakeState);

  }

  public static boolean getBrakeEnabled() {
    return brakeState;
  }

  /**
   * set the pivot voltage - positive values move towards the front of the robot.
   * @param volts -12 to 12
   */
  private void setVoltage(double volts){
    pivotMaster.setVoltage(volts);
  }

  /**
   * set the pivot angle - positive values move towards the front of the robot.
   * this enables feedback control.
   * TODO - the arm sim calls -90deg straight down - I will change this to 180 deg straight down.
   *    this function should call 180deg straight down. (zero degrees is straight up)
   *    what a mess. "I should have just used radians." (<--AI Wrote this)
   * @param angle a Rotation2d with the setpoint.
   */
  public void setAngle(Rotation2d angle){
    setpoint = angle.getRadians();
    enableLoop(this::setVoltage, this::getAngleRadians, this::getAngularVelocityRadiansPerSecond);
    goToState(angle.getRadians(), 0);
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
    var rotation = Units.rotationsToRadians(
            Util.countsToRotations(pivotMaster.getSelectedSensorPosition(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing)
    );
    return new Rotation2d(rotation);
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
    return Units.rotationsToRadians(
            Util.countsToRotations(pivotMaster.getSelectedSensorVelocity(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing)
    );
  }

  //SIMULATION: (this code is messy but I don't care)
  //NOTE: the pivot sim calls -90deg straight down (where gravity pull to).
  //I have to figure out how to make it call 0deg straight down.
  //TODO only initialize these in simulation
  // private final TalonFXSimCollection turretMotorSim = pivotMaster.getSimCollection();
  // private final static SingleJointedArmSim pivotSim = new SingleJointedArmSim(
  //         SYSTEM_CONSTANTS.motor,
  //         SYSTEM_CONSTANTS.gearing,
  //         SYSTEM_CONSTANTS.inertia,
  //         LENGTH,
  //         -100,
  //         100,
  //         MASS,
  //         true
  // );
  // private double prevAngleSetpoint = 0, prevPercentSetpoint = 0;
  private final boolean PERIODIC_CONTROL_SIMULATION = false;
  // @Override
  // public void simulationPeriodic() {
  //   //update the simulation
  //   pivotSim.setInputVoltage(pivotMaster.get() * RobotController.getBatteryVoltage());
  //   pivotSim.update(0.02);
  //   SmartDashboard.putNumber("pivot master output", pivotMaster.get());

  //   //write the simulation data to the motor sim
  //   //convert angle from sim to encoder tics:
  //   turretMotorSim.setIntegratedSensorRawPosition(
  //           (int) Util.rotationsToCounts(
  //                   //account for the fact that the sim calls -90deg straight down by adding 270deg
  //                   Units.radiansToRotations(pivotSim.getAngleRads() + (Math.PI * 1.5)),
  //                   SYSTEM_CONSTANTS.cpr,
  //                   SYSTEM_CONSTANTS.gearing
  //           )
  //   );
  //   turretMotorSim.setIntegratedSensorVelocity(
  //           (int) Util.rotationsToCounts(
  //                   Units.radiansToRotations(pivotSim.getVelocityRadPerSec()),
  //                   SYSTEM_CONSTANTS.cpr,
  //                   SYSTEM_CONSTANTS.gearing
  //           )
  //   );

  //   //get new angle setpoint from dashboard
  //   var angleSetpoint = SmartDashboard.getNumber("pivot setpoint angle", 0);
  //   if(angleSetpoint != prevAngleSetpoint && PERIODIC_CONTROL_SIMULATION){
  //     setAngle(Rotation2d.fromDegrees(angleSetpoint));
  //     prevAngleSetpoint = angleSetpoint;
  //   }
  //   //get new percent setpoint from dashboard
  //   var percentSetpoint = SmartDashboard.getNumber("pivot setpoint percent", 0);
  //   if(percentSetpoint != prevPercentSetpoint && PERIODIC_CONTROL_SIMULATION){
  //     setPercent(percentSetpoint);
  //     prevPercentSetpoint = percentSetpoint;
  //   }

  //   //update visualization
  //   DashboardManager.getInstance().drawPivotSim(Units.radiansToDegrees(pivotSim.getAngleRads()));

  //   //write information to dashboard:
  //   SmartDashboard.putNumber("pivot angle degrees", Math.toDegrees(getAngleRadians()));
  //   SmartDashboard.putNumber("pivot angular velocity", getAngularVelocityRadiansPerSecond());
  //   SmartDashboard.putNumber("pivot setpoint degrees", Math.toDegrees(setpoint));
  // }
}
