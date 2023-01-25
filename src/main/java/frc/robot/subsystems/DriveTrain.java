// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.sensors.PigeonHelper;
import frc.lib.util.DriveSignal;
import static frc.robot.Constants.DriveTrain.*;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(LEFT_MASTER);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(RIGHT_MASTER);
  private final WPI_TalonFX leftSlave = new WPI_TalonFX(LEFT_SLAVE);
  private final WPI_TalonFX rightSlave = new WPI_TalonFX(RIGHT_SLAVE);
  private final DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);

  private final SimpleMotorFeedforward feedforward;
  private final PIDController leftController;
  private final PIDController rightController;
  private final PigeonHelper pigeon;

//TODO  low gear make the robot go backwards so like, do something about it

  /** Creates a new DriveTrain. */
  public DriveTrain(PigeonHelper pigeon) {
    this.pigeon = pigeon;

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);

    leftMaster.setInverted(true);
    rightMaster.setInverted(false);

    leftSlave.setInverted(InvertType.FollowMaster);
    rightSlave.setInverted(InvertType.FollowMaster);

    setNeutralMode(NeutralMode.Brake);

    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    leftController = new PIDController(kP, kI, kD);
    rightController = new PIDController(kP, kI, kD);
  }

  public DifferentialDrive getDifferentialDrive(){
    return differentialDrive;
  }

  @Override
  public void periodic() {
    
  }

  public double countsToMeters(double encoderCounts){
    return ((encoderCounts / DRIVE_ENCODER_CPR) / GEARBOX_RATIO_HIGH) * METERS_PER_ROTATION;
  }

  public double metersToCounts(double meters){
    return ((meters / METERS_PER_ROTATION) * GEARBOX_RATIO_HIGH) * DRIVE_ENCODER_CPR;
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

  double dt = 0;
  //EXPERIMENTAL
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

  public double getLeftVelocity(){ //TODO Write Javadoc
    return countsToMeters(leftMaster.getSelectedSensorVelocity());
  }

  public double getRightVelocity(){ //TODO Write Javadoc
    return countsToMeters(rightMaster.getSelectedSensorVelocity());
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
  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
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

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

  //simulation
  private final TalonFXSimCollection leftMasterSim = new TalonFXSimCollection(leftMaster);
  private final TalonFXSimCollection rightMasterSim = new TalonFXSimCollection(rightMaster);

  public DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
    DCMotor.getFalcon500(2),
    GEARBOX_RATIO_HIGH,
    MOMENT_OF_INERTIA,
    MASS,
    Units.inchesToMeters(3), //wheel radius
    TRACK_WIDTH,

    // The standard deviations for measurement noise:
    //x meters, y meters, heading rad, l velocity m/s, r velocity m/s, l position m, r position m
    VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));
  @Override
  public void simulationPeriodic(){
    driveSim.setInputs(leftMaster.get() * RobotController.getBatteryVoltage(), rightMaster.get() * RobotController.getBatteryVoltage());
    driveSim.update(0.02);
    leftMasterSim.setIntegratedSensorRawPosition((int)metersToCounts(driveSim.getLeftPositionMeters()));
    rightMasterSim.setIntegratedSensorRawPosition((int)metersToCounts(driveSim.getRightPositionMeters()));
    leftMasterSim.setIntegratedSensorVelocity((int)metersToCounts(driveSim.getLeftVelocityMetersPerSecond()));
    rightMasterSim.setIntegratedSensorVelocity((int)metersToCounts(driveSim.getRightVelocityMetersPerSecond()));
    pigeon.setSimHeading(-driveSim.getHeading().getDegrees());


    
  }



}



