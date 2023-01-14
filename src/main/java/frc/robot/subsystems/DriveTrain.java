// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.DriveTrain.LEFT_MASTER;
import static frc.robot.Constants.DriveTrain.LEFT_SLAVE;
import static frc.robot.Constants.DriveTrain.RIGHT_MASTER;
import static frc.robot.Constants.DriveTrain.RIGHT_SLAVE;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constraints;
import frc.robot.RobotContainer;

public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX leftMaster = new WPI_TalonFX(LEFT_MASTER);
  private WPI_TalonFX leftSlave = new WPI_TalonFX(LEFT_SLAVE);
  private WPI_TalonFX rightMaster = new WPI_TalonFX(RIGHT_MASTER);
  private WPI_TalonFX rightSlave = new WPI_TalonFX(RIGHT_SLAVE);
  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);
  private Constraints constraints;
  private RobotContainer robot;



  /** Creates a new DriveTrain. */
  public DriveTrain(RobotContainer robot) {
    this.robot = robot;
    setNeutralMode(NeutralMode.Brake);
    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    leftMaster.setInverted(true);
    rightMaster.setInverted(false);
    leftSlave.setInverted(InvertType.FollowMaster); //TODO Confirm left bottom motor needs to oppose top master
    rightSlave.setInverted(InvertType.FollowMaster); //TODO Confirm right bottom motor needs to follow top master
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
   * input 2 doubles to drive the drivetrain motors seperatly
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
   * drive the robot in Arcade mode using the built in WPILib differential drive class
   * @param xForward joystick forward backward axis
   * @param zRotate joystick left right axis
   *
   */
  public void arcadeDrive(double forward, double wheel) {
    double forwardFiltered = robot.constrains.constrainJoystickFwdJerk(forward);
    tankDrive(new DriveSignal(forwardFiltered + wheel, forwardFiltered - wheel));
    differentialDrive.feed();
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



