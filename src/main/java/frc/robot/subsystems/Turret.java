// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Util;

import static frc.robot.Constants.Turret.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Turret extends SubsystemBase {
  private final WPI_TalonFX turretMaster = new WPI_TalonFX(MOTOR_PORT);
  private final LinearSystem<N2, N1, N2> turretPlant = LinearSystemId.createDCMotorSystem(
          DCMotor.getFalcon500(1),
          INERTIA, //TODO inertia of turret
          GEARING //TODO gearing of turret
  );
  private final KalmanFilter<N2, N1, N2> turretKalmanFilter = new KalmanFilter<>(
          Nat.N2(),
          Nat.N2(),
          turretPlant,
          VecBuilder.fill(KALMAN_MODEL_ACCURACY_POSITION, KALMAN_MODEL_ACCURACY_VELOCITY), //TODO model accuracy
          VecBuilder.fill(KALMAN_SENSOR_ACCURACY_POSITION, KALMAN_SENSOR_ACCURACY_VELOCITY), //TODO sensor accuracy
          DT
  );
  private final LinearQuadraticRegulator<N2, N1, N2> turretLQR = new LinearQuadraticRegulator<>(
          turretPlant,
          VecBuilder.fill(LQR_POSITION_TOLERANCE, LQR_VELOCITY_TOLERANCE), //TODO position/velocity error tolerance
          VecBuilder.fill(LQR_VOLTAGE_EFFORT), //voltage control effort
          DT
  );
  private final LinearSystemLoop<N2, N1, N2> turretLoop = new LinearSystemLoop<>(
          turretPlant,
          turretLQR,
          turretKalmanFilter,
          MAX_VOLTAGE,
          DT
  );
  private final TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(MAX_ANGULAR_VELOCITY, MAX_ANGULAR_ACCELERATION); //TODO
  private TrapezoidProfile profile = new TrapezoidProfile(
            constraints,
          new TrapezoidProfile.State(0, 0)
  );
  private boolean followingProfile = false;
  private final Timer profileTimer = new Timer();

  /** Creates a new Turret. */
  public Turret() {
    profileTimer.reset();
    turretMaster.setInverted(false);
  }

  @Override
  public void periodic() {
    //start the profile timer if the turret just started following a profile, otherwise, reset it
    if(followingProfile && profileTimer.get() == 0){
      profileTimer.start();
    } else if(!followingProfile){
      profileTimer.reset();
    }
    if(followingProfile){
      var time = profileTimer.get();
      var output = profile.calculate(time);
      turretLoop.setNextR(VecBuilder.fill(output.position, output.velocity));
      turretLoop.correct(VecBuilder.fill(getAngle().getRadians(), getAngularVelocity().getRadians()));
      turretLoop.predict(DT);
      setVoltageWithoutStoppingProfile(turretLoop.getU(0));
    }
  }

  /**
   * set the turret speed, Positive Values should be Counter Clock Wise
   */
  public void setPercent(double speed) {
    followingProfile = false;
    turretMaster.set(speed);
  }

  public void setVoltage(double volts){
    followingProfile = false;
    setVoltageWithoutStoppingProfile(volts);
  }
  private void setVoltageWithoutStoppingProfile(double volts){
    turretMaster.setVoltage(volts);
  }

  public void setProfile(TrapezoidProfile.State goal){
    var current = new TrapezoidProfile.State(getAngle().getDegrees(), getAngularVelocity().getDegrees());
    profile = new TrapezoidProfile(constraints, goal, current);
    followingProfile = true;
  }

  /**
   * set the angle of the turret with a {@link Rotation2d}
   * 0 is the front of the robot, positive values
   * turn the turret counterclockwise
   * @param angle {@link Rotation2d}
   */
  public void setAngle(Rotation2d angle){
    var goal = new TrapezoidProfile.State(angle.getDegrees(), 0);
    setProfile(goal);
  }

  /**
   * get the current angle of the turret
   * @return {@link Rotation2d}
   */
  public Rotation2d getAngle(){
    var rotation = Util.countsToRotations(turretMaster.getSelectedSensorPosition(), ENCODER_CPR, GEARING); //todo  Gear Ratiow
    return Rotation2d.fromDegrees(rotation*360);
  }
  public Rotation2d getAngularVelocity(){
    var velocity = Util.countsToRotations(turretMaster.getSelectedSensorVelocity(), ENCODER_CPR, GEARING); //todo  Gear Ratiow
    return Rotation2d.fromDegrees(velocity*360);
  }
}
