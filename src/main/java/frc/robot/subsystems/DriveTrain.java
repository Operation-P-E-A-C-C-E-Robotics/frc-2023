// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DriveSignal;
import frc.robot.Constraints;
import frc.robot.RobotContainer;

import static frc.robot.Constants.Auto.*;
import static frc.robot.Constants.DriveTrain.*;

public class DriveTrain extends SubsystemBase {
  private final WPI_TalonFX leftMaster = new WPI_TalonFX(LEFT_MASTER);
  private final WPI_TalonFX rightMaster = new WPI_TalonFX(RIGHT_MASTER);
  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
  private Constraints constraints;
  private final RobotContainer robot;

  private final SimpleMotorFeedforward feedforward;
  private final PIDController leftController, rightController;



  /** Creates a new DriveTrain. */
  public DriveTrain(RobotContainer robot) {
    this.robot = robot;
    setNeutralMode(NeutralMode.Brake);
    WPI_TalonFX leftSlave = new WPI_TalonFX(LEFT_SLAVE);
    leftSlave.follow(leftMaster);
    WPI_TalonFX rightSlave = new WPI_TalonFX(RIGHT_SLAVE);
    rightSlave.follow(rightMaster);
    leftMaster.setInverted(true);
    rightMaster.setInverted(false);
    leftSlave.setInverted(InvertType.FollowMaster); //TODO Confirm left bottom motor needs to oppose top master
    rightSlave.setInverted(InvertType.FollowMaster); //TODO Confirm right bottom motor needs to follow top master

    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
    leftController = new PIDController(kP, kI, kD);
    rightController = new PIDController(kP, kI, kD);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

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

  public void tankDrive(DriveSignal signal){
    tankDrive(signal.getLeft(), signal.getRight());
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
    double forwardFiltered = robot.constrains.constrainJoystickFwdJerk(forward);
    tankDrive(new DriveSignal(forwardFiltered + wheel, forwardFiltered - wheel));
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

  public void velocityDriveHold(DifferentialDriveWheelSpeeds speeds, DifferentialDriveWheelSpeeds previousSpeeds, double dt, double tilt){
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
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorVelocity(), rightMaster.getSelectedSensorVelocity()); //TODO confirm this returns correct data
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
    return (getLeftEncoder() + getRightEncoder() / 2.0);
  }



  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoder() {
    return leftMaster.getSelectedSensorPosition(); //TODO convert to Meters
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoder() {
    return leftMaster.getSelectedSensorPosition(); //TODO convert to Meters
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    differentialDrive.setMaxOutput(maxOutput);
  }

}



