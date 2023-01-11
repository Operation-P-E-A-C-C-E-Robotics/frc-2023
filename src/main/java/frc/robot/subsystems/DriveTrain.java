// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.DriveTrain.*;
 
public class DriveTrain extends SubsystemBase {
  private WPI_TalonFX leftMaster = new WPI_TalonFX(LEFT_MASTER);
  private WPI_TalonFX leftSlave = new WPI_TalonFX(LEFT_SLAVE);
  private WPI_TalonFX rightMaster = new WPI_TalonFX(RIGHT_MASTER);
  private WPI_TalonFX rightSlave = new WPI_TalonFX(RIGHT_SLAVE);
  private DifferentialDrive differentialDrive = new DifferentialDrive(leftMaster, rightMaster);



  /** Creates a new DriveTrain. */
  public DriveTrain() {
    setNeutralMode(NeutralMode.Brake);
    leftSlave.follow(leftMaster);
    leftSlave.setInverted(InvertType.OpposeMaster); //TODO Confirm left bottom motor needs to oppose top master
    rightSlave.follow(rightMaster);
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
    leftSlave.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    rightSlave.setNeutralMode(mode);
  }
  /**
   * input 2 doubles to drive the drivetrain motors seperatly
   * @param leftSpeed the speed to set the left motors to (Double)
   * @param rightSpeed the speed to set the right motors to (Double)
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    leftMaster.set(leftSpeed);
    rightMaster.set(rightSpeed);
  }

  /**
   * drive the robot in Arcade mode using the built in WPILib differential drive class
   * @param xForward joystick forward backward axis
   * @param zRotate joystick left right axis
   * 
   */
  public void arcadeDrive(double xForward, double zRotate) {
    differentialDrive.arcadeDrive(xForward, zRotate);
  }


  //WPILib built in odometry methods from docs



  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftMaster.getSelectedSensorPosition(), rightMaster.getSelectedSensorPosition()); //TODO confirm this returns correct data
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public String resetOdometry(Pose2d pose) {
    String incomplete = new String("This function is not implimented");
   return incomplete; //remember to reset it to void when we write this
  }





  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    // return (leftMaster.get + m_rightEncoder.getDistance()) / 2.0;
   DriverStation.reportError("method ''getAverageEncoderDistance'' not implimented, garbage value returned", false); //TODO write this
    return 0;
  }



  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoder() {
    return leftMaster.getSelectedSensorPosition();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoder() {
    return leftMaster.getSelectedSensorPosition();
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



