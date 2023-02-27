// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.EndEffector.*;

public class EndEffector extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);

  private final DoubleSolenoid clawSolenoid = new DoubleSolenoid(6, PneumaticsModuleType.REVPH, GRIP_OPEN_PNEUMATICS_PORT, GRIP_CLOSED_PNEUMATICS_PORT); //TODO
  
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch colorMatcher = new ColorMatch();
  private final DigitalInput beamBrakeSensor = new DigitalInput(BEAM_BRAKE_PORT); //TODO

  private IntakeState state = IntakeState.RESTING;
  private Timer ejectTimer = new Timer();
  private Timer clawTimer = new Timer();

  /** Creates a new Intake. */
  public EndEffector() {
    leftMotor.setInverted(false);
    rightMotor.setInverted(false);

    colorMatcher.addColorMatch(CUBE_COLOR); //TODO
    colorMatcher.addColorMatch(CONE_COLOR); //TODO
  }

  @Override
  public void periodic() {
    // Update the intake state using data from the color sensor and beam brake sensor
    var color = colorMatcher.matchClosestColor(colorSensor.getColor());
    var beamBroken = beamBrakeSensor.get();
  
    var intaking = leftMotor.get() > 0.1; //TODO threshold
    var ejecting = leftMotor.get() < -0.1;

    if(intaking){
      //if the motors are intaking
      if(color.confidence < 0.5) { //TODO threshold
        //the color sensor sees something, so we now have something:
        if(color.color == CONE_COLOR) state = IntakeState.HAS_CONE;
        else state = IntakeState.HAS_CUBE;
      } else if (beamBroken){
        //the beam brake is broken so we have something in the claw
        state = IntakeState.GETTING_OBJECT;
      } else {
        //we don't have shit and we just spinning the wheels.
        state = IntakeState.INTAKING;
      }
    } else if (ejecting) {
      //check if we're ejecting a cone or a cube:
      if(state == IntakeState.HAS_CONE) state = IntakeState.EJECTING_CONE;
      else if(state == IntakeState.HAS_CUBE) state = IntakeState.EJECTING_CUBE;
      else if(!ejecting()) state = IntakeState.EJECTING_UNKNOWN; //we haven't detected a cone or a cube yet

      //start the timer once the object is out of the beam brake:
      if(!beamBroken) ejectTimer.start();
      if(ejectTimer.get() > TIME_TO_EJECT) state = IntakeState.EJECTING_NOTHING;
    } else {
      //we aren't spinning the wheels
      if(state != IntakeState.HAS_CONE && state != IntakeState.HAS_CUBE){
        //we're just chillin:
        state = IntakeState.RESTING;
      }
    }
    if(!ejecting){
      ejectTimer.stop();
      ejectTimer.reset();
    }
  }

  /**
   * spin the intake, 1 should intake and -1 should spit out
   */
  public void setPercent(double speed) {
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  /**
   * set the state of the intake pneumatics. There are three states:
   * @param open true to open the claw, false to close it
   */
  public void setClaw(boolean open){
    var newDirection = open ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse;
    clawSolenoid.set(newDirection);
    if(newDirection != clawSolenoid.get()) {
      clawTimer.reset();
      clawTimer.start();
    }
  }

  public boolean isClawOpen(){
    return clawSolenoid.get() == DoubleSolenoid.Value.kForward && clawTimer.get() > TIME_FOR_CLAW_TO_OPEN;
  }

  public boolean isClawClosed(){
    return clawSolenoid.get() == DoubleSolenoid.Value.kReverse && clawTimer.get() > TIME_FOR_CLAW_TO_OPEN;
  }

  public boolean hasCube(){
    return state == IntakeState.HAS_CUBE || state == IntakeState.EJECTING_CUBE;
  }

  public boolean hasCone(){
    return state == IntakeState.HAS_CONE || state == IntakeState.EJECTING_CONE;
  }

  public boolean ejecting(){
    return state == IntakeState.EJECTING_CONE || state == IntakeState.EJECTING_CUBE || state == IntakeState.EJECTING_NOTHING;
  }

  public boolean beamBroken(){
    return beamBrakeSensor.get();
  }

  public IntakeState getState(){
    return state;
  }

  public enum IntakeState{
    INTAKING,
    GETTING_OBJECT,
    HAS_CONE,
    HAS_CUBE,
    EJECTING_CONE,
    EJECTING_CUBE,
    EJECTING_UNKNOWN,
    EJECTING_NOTHING,
    RESTING,
  }
}
