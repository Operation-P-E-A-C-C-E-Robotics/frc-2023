// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.safety.DankPids;
import frc.lib.sensors.PigeonHelper;
import frc.lib.util.DriveSignal;
import frc.robot.Constants;

import static frc.robot.Constants.DriveTrain.*;

public class DriveTrain extends SubsystemBase {
  //drive motor controllers:
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(LEFT_MASTER);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(RIGHT_MASTER);
  private final WPI_TalonFX leftSlave = new WPI_TalonFX(LEFT_SLAVE);
  private final WPI_TalonFX rightSlave = new WPI_TalonFX(RIGHT_SLAVE);

  //velocity drive
  //with current auto setup.
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
  private final PigeonHelper pigeon;
  private final Timer shiftClutchTimer = new Timer();
  private boolean shiftClutchEngaged = false;

  //SHIFTING!:
  private final DoubleSolenoid shiftSolenoid = new DoubleSolenoid(
          Constants.LOWER_PNEUMATICS_MODULE_CAN_ID,
          PneumaticsModuleType.REVPH,
          SHIFT_HIGH_PORT,
          SHIFT_LOW_PORT
  );
  private boolean isBrakeMode = false;

  private final DriveVelocityController highVelocityController = new DriveVelocityController(kV_LINEAR, kA_LINEAR, kV_ANGULAR, kA_ANGULAR);
  private final DriveVelocityController lowVelocityController = new DriveVelocityController(kV_LINEAR_LOW, kA_LINEAR_LOW, kV_ANGULAR_LOW, kA_ANGULAR_LOW);

  private double leftPositionOffset, rightPositionOffset; //to keep position consistent across gears

  private double leftVelocitySetpoint = 0, rightVelocitySetpoint = 0;
  private boolean isClosedLoop = false;

  private static final Value HIGH_GEAR = Value.kForward;
  private static final Value LOW_GEAR = Value.kReverse;

  /** Creates a new DriveTrain. */
  public DriveTrain(PigeonHelper pigeon) {
    this.pigeon = pigeon;

    //configure motor controllers:
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.configFactoryDefault();
    rightMaster.configFactoryDefault();
    leftSlave.configFactoryDefault();
    rightSlave.configFactoryDefault();

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    leftMaster.setInverted(Constants.Inversions.DRIVE_LEFT);
    rightMaster.setInverted(Constants.Inversions.DRIVE_RIGHT);
    // set(new DriveSignal(0, 0, false, false));
    set(DriveSignal.DEFAULT);

    //configure PID

    DankPids.registerDankTalon(leftMaster);
    DankPids.registerDankTalon(rightMaster);
    DankPids.registerDankTalon(leftSlave);
    DankPids.registerDankTalon(rightSlave);
  }


  public void set(DriveSignal signal) {
    setHighGear(signal.isHighGear());
    setBrakeMode(signal.isBrakeMode());

    // if(shiftClutchTimer.get() < 0.2) return;

    switch(signal.getControlMode()){
      case OPEN_LOOP -> {
        setPercent(signal.getLeft(), signal.getRight());
        isClosedLoop = false;
      }
      case VOLTAGE -> {
        setVoltage(signal.getLeft(), signal.getRight());
        isClosedLoop = false;
      }
      case VELOCITY -> {
        leftVelocitySetpoint = signal.getLeft();
        rightVelocitySetpoint = signal.getRight();
        isClosedLoop = true;
      }
    }
    differentialDrive.feed();
  }

  private void setVoltage(double leftVoltage, double rightVoltage){
    leftMaster.setVoltage(leftVoltage);
    rightMaster.setVoltage(rightVoltage);
  }

  private void setPercent(double leftPercent, double rightPercent){
    leftMaster.set(leftPercent);
    rightMaster.set(rightPercent);
  }

  public void resetVelocityDrive(){
    highVelocityController.loop.reset(VecBuilder.fill(getLeftVelocity(), getRightVelocity()));
  }

  //WPILib built in odometry methods from docs

  /**
   * get the velocity of the left side of the drivetrain
   * @return the velocity of the left side of the drivetrain in meters per second
   */
  public double getLeftVelocity(){
    return countsToMeters(leftMaster.getSelectedSensorVelocity() * 10); //times 10 to convert units/100ms to units/sec
  }

  /**
   * get the velocity of the right side of the drivetrain
   * @return the velocity of the right side of the drivetrain in meters per second
   */
  public double getRightVelocity(){
    return countsToMeters(rightMaster.getSelectedSensorVelocity() * 10); //times 10 to convert units/100ms to units/sec
  }

  public double getAverageVelocity(){
    return (getLeftVelocity() + getRightVelocity()) / 2;
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders(double left, double right) {
    leftMaster.setSelectedSensorPosition(left);
    rightMaster.setSelectedSensorPosition(right);
  }

  /**
   * Gets the average distance of the two master encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftMeters() + getRightMeters() / 2.0);
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftMeters() {
    return countsToMeters(leftMaster.getSelectedSensorPosition());
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightMeters() {
    return countsToMeters(rightMaster.getSelectedSensorPosition());
  }

  public DifferentialDrive getDifferentialDrive(){
    return differentialDrive;
  }

  public double countsToMeters(double encoderCounts){
    return ((encoderCounts / DRIVE_ENCODER_CPR) / getCurrentGearRatio()) * WHEEL_CIRCUMFERENCE;
  }

  public double countsToMeters(double encoderCounts, boolean isHighGear){
    return ((encoderCounts / DRIVE_ENCODER_CPR) / (isHighGear ? GEARBOX_RATIO_HIGH : GEARBOX_RATIO_LOW)) * WHEEL_CIRCUMFERENCE;
  }

  public double metersToCounts(double meters){
    return ((meters / WHEEL_CIRCUMFERENCE) * getCurrentGearRatio()) * DRIVE_ENCODER_CPR;
  }

  private void setHighGear(boolean isHighGear){
    // if(isHighGear == isHighGear()){
      leftPositionOffset = getLeftMeters() - countsToMeters(leftMaster.getSelectedSensorPosition(), isHighGear);
      rightPositionOffset = getRightMeters() - countsToMeters(rightMaster.getSelectedSensorPosition(), isHighGear);

      shiftSolenoid.set(isHighGear ? HIGH_GEAR : LOW_GEAR);

      shiftClutchTimer.reset();
      shiftClutchTimer.start();

      shiftClutchEngaged = false;

      leftMaster.setInverted(isHighGear);
      rightMaster.setInverted(!isHighGear);
    // }
  }

  private void setBrakeMode(boolean isBrakeMode){
    if(isBrakeMode != this.isBrakeMode){
      leftMaster.setNeutralMode(isBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
      rightMaster.setNeutralMode(isBrakeMode ? NeutralMode.Brake : NeutralMode.Coast);
    }
  }

  public boolean isHighGear(){
    return shiftSolenoid.get() != HIGH_GEAR;
  }

  public double getCurrentGearRatio(){
    return isHighGear() ? GEARBOX_RATIO_HIGH : GEARBOX_RATIO_LOW;
  }

  @Override
  public void periodic(){
    SmartDashboard.putBoolean("HIGH GEAR", isHighGear());
    if(isClosedLoop){
      var loop = isHighGear() ? highVelocityController.loop : lowVelocityController.loop;

      loop.setNextR(VecBuilder.fill(leftVelocitySetpoint, rightVelocitySetpoint));
      loop.correct(VecBuilder.fill(getWheelSpeeds().leftMetersPerSecond, getWheelSpeeds().rightMetersPerSecond));
      loop.predict(0.02);

      var left = loop.getU(0);
      var right = loop.getU(1);
      setVoltage(left, right);
    }
    //current limit while shifting to prevent damage, but only configure 1 time per shift.
    // if(shiftClutchTimer.get() < 0.1 && !shiftClutchEngaged){
    //   leftMaster.configStatorCurrentLimit(SHIFTING_CURRENT_LIMIT);
    //   rightMaster.configStatorCurrentLimit(SHIFTING_CURRENT_LIMIT);
    //   shiftClutchEngaged = true;
    // }
    // if(shiftClutchTimer.get() > 0.1 && shiftClutchEngaged){
    //   leftMaster.configStatorCurrentLimit(CURRENT_LIMIT);
    //   rightMaster.configStatorCurrentLimit(CURRENT_LIMIT);
    //   shiftClutchEngaged = false;
    // }
    SmartDashboard.putData("drivetrain", differentialDrive);
  }

  //simulation
  private final TalonFXSimCollection leftMasterSim = new TalonFXSimCollection(leftMaster);
  private final TalonFXSimCollection rightMasterSim = new TalonFXSimCollection(rightMaster);

  public DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
          highVelocityController.drivePlant,
    DCMotor.getFalcon500(4),
    1/(GEARBOX_RATIO_HIGH),
    TRACK_WIDTH,
    Units.inchesToMeters(3),
    // The standard deviations for measurement noise:
    //x meters, y meters, heading rad, l velocity m/s, r velocity m/s, l position m, r position m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  @Override
  public void simulationPeriodic(){
    driveSim.setInputs(rightMaster.get() * RobotController.getBatteryVoltage(), leftMaster.get() * RobotController.getBatteryVoltage());
    driveSim.update(0.02);
    leftMasterSim.setIntegratedSensorRawPosition((int)metersToCounts(driveSim.getRightPositionMeters()));
    rightMasterSim.setIntegratedSensorRawPosition(-(int)metersToCounts(driveSim.getLeftPositionMeters()));
    leftMasterSim.setIntegratedSensorVelocity((int)metersToCounts(driveSim.getRightVelocityMetersPerSecond()));
    rightMasterSim.setIntegratedSensorVelocity(-(int)metersToCounts(driveSim.getLeftVelocityMetersPerSecond()));
    pigeon.setSimHeading(driveSim.getHeading().getDegrees());
  }

  public static class DriveVelocityController{
    public final LinearSystem<N2, N2, N2> drivePlant;
    public final LinearQuadraticRegulator<N2, N2, N2> driveLQR;
    public final KalmanFilter<N2, N2, N2> kalmanFilter;
    public final LinearSystemLoop<N2, N2, N2> loop;

    public DriveVelocityController(double kVLinear, double kALinear, double kVAngular, double kAAngular){
        drivePlant = LinearSystemId.identifyDrivetrainSystem(
                kVLinear,
                kALinear,
                kVAngular,
                kAAngular,
                TRACK_WIDTH
        );
        driveLQR = new LinearQuadraticRegulator<>(
                drivePlant,
                VecBuilder.fill(LQR_ERROR_TOLERANCE, LQR_ERROR_TOLERANCE),
                VecBuilder.fill(LQR_EFFORT, LQR_EFFORT),
                DT
        );
        kalmanFilter = new KalmanFilter<>(
                Nat.N2(),
                Nat.N2(),
                drivePlant,
                VecBuilder.fill(KALMAN_MODEL_ACCURACY, KALMAN_MODEL_ACCURACY),
                VecBuilder.fill(KALMAN_SENSOR_ACCURACY, KALMAN_SENSOR_ACCURACY),
                DT
        );
        loop = new LinearSystemLoop<>(
                drivePlant,
                driveLQR,
                kalmanFilter,
                12,
                DT
        );
    }
  }
}