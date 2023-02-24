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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.safety.DankPids;
import frc.lib.safety.RedundantSystem;
import frc.lib.sensors.PigeonHelper;
import frc.lib.util.DriveSignal;
import static frc.robot.Constants.DriveTrain.*;

public class DriveTrain extends SubsystemBase {
  //drive motor controllers:
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(LEFT_MASTER);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(RIGHT_MASTER);
  private final WPI_TalonFX leftSlave = new WPI_TalonFX(LEFT_SLAVE);
  private final WPI_TalonFX rightSlave = new WPI_TalonFX(RIGHT_SLAVE);

  //fancy redundancy stuff:
  private final RedundantSystem<Double> leftPosition, rightPosition;

  //velocity drive - TODO if LQR velocity drive works, get rid of pid. also we may not need differential drive
  //with current auto setup.
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
  private final SimpleMotorFeedforward feedforward;
  private final PIDController leftController;
  private final PIDController rightController;
  private final PigeonHelper pigeon;

  //SHIFTING!:
  private final DoubleSolenoid shiftSolenoid = new DoubleSolenoid(6, PneumaticsModuleType.REVPH, SHIFT_HIGH_PORT, SHIFT_LOW_PORT);
  private Gear gear = Gear.HIGH;

  //LQR velocity drive:
  //STATES: [left velocity, right velocity]
  //INPUTS: [left voltage, right voltage]
  //OUTPUTS: [left velocity, right velocity]
  private final LinearSystem<N2, N2, N2> drivePlant = LinearSystemId.identifyDrivetrainSystem(kV, kA, 2, 0.2, TRACK_WIDTH); //TODO angular kV and kA
  private final LinearQuadraticRegulator<N2, N2, N2> driveLQR = new LinearQuadraticRegulator<>(
          drivePlant,
          VecBuilder.fill(LQR_ERROR_TOLERANCE, LQR_ERROR_TOLERANCE),
          VecBuilder.fill(LQR_EFFORT, LQR_EFFORT),
          DT
  );
  private final KalmanFilter<N2, N2, N2> kalmanFilter = new KalmanFilter<>(
          Nat.N2(),
          Nat.N2(),
          drivePlant,
          VecBuilder.fill(KALMAN_MODEL_ACCURACY, KALMAN_MODEL_ACCURACY),
          VecBuilder.fill(KALMAN_SENSOR_ACCURACY, KALMAN_SENSOR_ACCURACY),
          DT
  );
  private final LinearSystemLoop<N2, N2, N2> loop = new LinearSystemLoop<>(
          drivePlant,
          driveLQR,
          kalmanFilter,
          12, //todo
          DT
  );


//TODO  low gear make the robot go backwards so like, do something about it

  /** Creates a new DriveTrain. */
  public DriveTrain(PigeonHelper pigeon) {
    this.pigeon = pigeon;

    //configure motor controllers:
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);
    leftMaster.setInverted(false);
    rightMaster.setInverted(true);

    setNeutralMode(NeutralMode.Brake);

    leftPosition = new RedundantSystem<>(
            new RedundantSystem.MultiCheck(
                    new RedundantSystem.SlewRateCheck(10000),
                    (Double value, boolean failed) -> failed || leftMaster.isAlive()
            ),
            leftMaster::getSelectedSensorPosition,
            leftSlave::getSelectedSensorPosition
    );
    rightPosition = new RedundantSystem<>(
            new RedundantSystem.MultiCheck(
                    new RedundantSystem.SlewRateCheck(10000),
                    (Double value, boolean failed) -> failed || rightMaster.isAlive()
            ),
            rightMaster::getSelectedSensorPosition,
            rightSlave::getSelectedSensorPosition
    );

    //configure PID: TODO get rid of PID
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    leftController = new PIDController(kP, kI, kD);
    rightController = new PIDController(kP, kI, kD);

    driveLQR.latencyCompensate(drivePlant, 0.02, 0); //TODO
    DankPids.registerDankTalon(leftMaster);
    DankPids.registerDankTalon(rightMaster);
    DankPids.registerDankTalon(leftSlave);
    DankPids.registerDankTalon(rightSlave);
  }

  /**
   * input 2 doubles to drive the drivetrain motors separately
   * @param leftSpeed the speed to set the left motors to (Double)
   * @param rightSpeed the speed to set the right motors to (Double)
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftMaster.set(leftSpeed);
    rightMaster.set(rightSpeed);
    differentialDrive.feed();
  }


  public void tankDrive(DriveSignal cheesyDrive) {
    tankDrive(cheesyDrive.getLeft(), cheesyDrive.getRight());
  }

  /**
   * drive the motors at a specific voltage
   * see {@link com.ctre.phoenix.motorcontrol.can.WPI_TalonFX#setVoltage(double)} for more details
   * this is NOT a set and forget function!
   * @param leftVolts voltage to send to the left side of the drive train
   * @param rightVolts voltage to send to the right side of the drive train
   */
  public void tankDriveVolts(double leftVolts, double rightVolts){
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(rightVolts);
        differentialDrive.feed();
  }

  /**
   * drive the robot in Arcade mode using the built-in WPILib differential drive class
   * @param forward joystick forward backward axis
   * @param wheel joystick left right axis
   *
   */
  public void arcadeDrive(double forward, double wheel) {
    tankDrive(new DriveSignal(forward + wheel, forward - wheel));
    differentialDrive.feed();
  }

  public void resetVelocityDrive(){
    leftController.reset();
    rightController.reset();
  }

  //EXPERIMENTAL
  public void velocityDriveLQR(DifferentialDriveWheelSpeeds speeds){
    //use the LQR to calculate the voltages needed to get to the desired speeds
    loop.setNextR(VecBuilder.fill(speeds.leftMetersPerSecond, speeds.rightMetersPerSecond));
    loop.correct(VecBuilder.fill(getWheelSpeeds().leftMetersPerSecond, getWheelSpeeds().rightMetersPerSecond));
    loop.predict(0.02);
    //set the voltages
    tankDriveVolts(loop.getU(0), loop.getU(1));
  }

  public void velocityDrive(DifferentialDriveWheelSpeeds speeds, DifferentialDriveWheelSpeeds previousSpeeds, double dt){
    double leftFeedforward, rightFeedforward, left, right;

    leftFeedforward = feedforward.calculate(
      speeds.leftMetersPerSecond,
      (speeds.leftMetersPerSecond - previousSpeeds.leftMetersPerSecond) / dt
    );

    rightFeedforward = feedforward.calculate(
      speeds.rightMetersPerSecond,
      (speeds.rightMetersPerSecond - previousSpeeds.rightMetersPerSecond) / dt
    );

    left = leftFeedforward + leftController.calculate(
            getWheelSpeeds().leftMetersPerSecond, speeds.leftMetersPerSecond
    );
    right = rightFeedforward + rightController.calculate(
            getWheelSpeeds().rightMetersPerSecond, speeds.rightMetersPerSecond
    );

    tankDriveVolts(left, right);
  }

  public void velocityDriveHold(DifferentialDriveWheelSpeeds speeds, DifferentialDriveWheelSpeeds previousSpeeds, double dt, double tilt){ //TODO Write Javadoc
    double leftFeedforward, rightFeedforward, left, right;

    leftFeedforward = feedforward.calculate(
            speeds.leftMetersPerSecond,
            (speeds.leftMetersPerSecond - previousSpeeds.leftMetersPerSecond) / dt
    ) + (kA * Math.sin(tilt));

    rightFeedforward = feedforward.calculate(
            speeds.rightMetersPerSecond,
            (speeds.rightMetersPerSecond - previousSpeeds.rightMetersPerSecond) / dt
    ) + (kA * Math.sin(tilt));

    left = leftFeedforward + leftController.calculate(
            getWheelSpeeds().leftMetersPerSecond, speeds.leftMetersPerSecond
    );
    right = rightFeedforward + rightController.calculate(
            getWheelSpeeds().rightMetersPerSecond, speeds.rightMetersPerSecond
    );

    tankDriveVolts(left, right);
  }

  //WPILib built in odometry methods from docs

  /**
   * get the velocity of the left side of the drivetrain
   * @return the velocity of the left side of the drivetrain in meters per second
   */
  public double getLeftVelocity(){
    return countsToMeters(leftMaster.getSelectedSensorVelocity());
  }

  /**
   * get the velocity of the right side of the drivetrain
   * @return the velocity of the right side of the drivetrain in meters per second
   */
  public double getRightVelocity(){
    return countsToMeters(rightMaster.getSelectedSensorVelocity());
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
   * set the drivetrain motors into a specific mode IE "Break Mode" only works for CTRE motors
   * @param mode com.ctre.phoenix.motorcontrol.NeuteralMode the mode to set the motors to
   *
   */
  public void setNeutralMode(NeutralMode mode) {
    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
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
    return countsToMeters(leftPosition.get());
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightMeters() {
    return countsToMeters(rightPosition.get());
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  public DifferentialDrive getDifferentialDrive(){
    return differentialDrive;
  }

  public double countsToMeters(double encoderCounts){
    return ((encoderCounts / DRIVE_ENCODER_CPR) / getCurrentGearRatio()) * METERS_PER_ROTATION;
  }

  public double metersToCounts(double meters){
    return ((meters / METERS_PER_ROTATION) * getCurrentGearRatio()) * DRIVE_ENCODER_CPR;
  }

  public void setGear(Gear gear){
    if(gear == this.gear) return;
    this.gear = gear;
    if(gear == Gear.LOW){
      shiftSolenoid.set(Value.kReverse);
      leftMaster.setInverted(true);
      rightMaster.setInverted(false);
    } else {
      leftMaster.setInverted(false);
      rightMaster.setInverted(true);
      shiftSolenoid.set(Value.kForward);
    }
  }

  public Gear getGear(){
    return gear;
  }

  public double getCurrentGearRatio(){
    if(gear == Gear.LOW){
      return GEARBOX_RATIO_LOW;
    } else {
      return GEARBOX_RATIO_HIGH;
    }
  }

  //simulation
  private final TalonFXSimCollection leftMasterSim = new TalonFXSimCollection(leftMaster);
  private final TalonFXSimCollection rightMasterSim = new TalonFXSimCollection(rightMaster);

  public DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
    drivePlant,
    DCMotor.getFalcon500(4),
    1/GEARBOX_RATIO_HIGH,
    TRACK_WIDTH,
    Units.inchesToMeters(3),
    // The standard deviations for measurement noise:
    //x meters, y meters, heading rad, l velocity m/s, r velocity m/s, l position m, r position m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  @Override
  public void simulationPeriodic(){
    driveSim.setInputs(leftMaster.get() * RobotController.getBatteryVoltage(), rightMaster.get() * RobotController.getBatteryVoltage());
    driveSim.update(0.02);
    leftMasterSim.setIntegratedSensorRawPosition(-(int)metersToCounts(driveSim.getRightPositionMeters()));
    rightMasterSim.setIntegratedSensorRawPosition((int)metersToCounts(driveSim.getLeftPositionMeters()));
    leftMasterSim.setIntegratedSensorVelocity(-(int)metersToCounts(driveSim.getRightVelocityMetersPerSecond()));
    rightMasterSim.setIntegratedSensorVelocity((int)metersToCounts(driveSim.getLeftVelocityMetersPerSecond()));
    pigeon.setSimHeading(driveSim.getHeading().getDegrees());
  }

  public enum Gear{
    HIGH,
    LOW
  }
}



