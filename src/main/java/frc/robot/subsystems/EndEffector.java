// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import java.awt.*;

import static frc.robot.Constants.EndEffector.*;

public class EndEffector extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);

  private final Solenoid clawSolenoid = new Solenoid(
          Constants.UPPER_PNEUMATICS_MODULE_CAN_ID,
          PneumaticsModuleType.CTREPCM,
          GRIP_PNEUMATICS_PORT
  );
  
  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch colorMatcher = new ColorMatch();
  private final Notifier colorSensorIsPiecaShit;

  private Color colorSensorReading = new Color(0,0,0);
  private double colorSensorProximity = 0;

  private IntakeState state = IntakeState.RESTING;
  private Timer ejectTimer = new Timer();
  private Timer clawTimer = new Timer();

  PhotonicHRI.PhotonicLingualElement ejectCone = RobotContainer.photonicHRI.blink(255, 177, 0, 0.1);
  PhotonicHRI.PhotonicLingualElement ejectCube = RobotContainer.photonicHRI.blink(0, 0, 255, 0.1);
  PhotonicHRI.PhotonicLingualElement ejectUnknown = RobotContainer.photonicHRI.blink(255, 0, 0, 0.1);
  PhotonicHRI.PhotonicLingualElement ejectNothing = RobotContainer.photonicHRI.blink(0, 255, 0, 0.1);

  /** Creates a new Intake. */
  public EndEffector() {
    leftMotor.setInverted(Constants.Inversions.INTAKE_LEFT);
    rightMotor.setInverted(Constants.Inversions.INTAKE_RIGHT);
    // leftMotor.setSmartCurrentLimit(10);
    leftMotor.setSmartCurrentLimit(10, 20);
    // rightMotor.setSmartCurrentLimit(10);
    rightMotor.setSmartCurrentLimit(10, 20);
    leftMotor.setOpenLoopRampRate(0.3);
    rightMotor.setOpenLoopRampRate(0.3);

    colorMatcher.addColorMatch(CUBE_COLOR); //TODO
    colorMatcher.addColorMatch(CONE_COLOR); //TODO

    colorSensorIsPiecaShit = new Notifier(() -> {
      colorSensorReading = colorSensor.getColor();
      colorSensorProximity = colorSensor.getProximity();
    });
    colorSensorIsPiecaShit.startPeriodic(0.02);
  }

  public boolean colorSensorSeesThing(){
    return colorSensorProximity > (isClawOpen() ? 44 : 90);
  }

  @Override
  public void periodic() {
    // Update the intake state using data from the color sensor and beam brake sensor
    var color = colorMatcher.matchClosestColor(colorSensorReading);
    var hasGamepiece = colorSensorSeesThing();
    SmartDashboard.putBoolean("Has gamepiece", hasGamepiece);
    SmartDashboard.putString("Intake state", state.toString());

    var ejecting = leftMotor.get() > 0.3;
    var intaking = leftMotor.get() < -0.3;

    SmartDashboard.putNumber("color snessor proximity", colorSensorProximity);
    SmartDashboard.putNumber("asomething", clawTimer.get());

    SmartDashboard.putBoolean("aisCone", color.color.equals(CONE_COLOR));
    SmartDashboard.putNumber("aconfidence", color.confidence);

     if(intaking){
       //if the motors are intaking
       if(hasGamepiece && color.confidence > 0.44) {
         //the color sensor sees something, so we now have sopmething:
         if(color.color.equals(CONE_COLOR)) state = IntakeState.HAS_CONE;
         else state = IntakeState.HAS_CUBE;
       } else if (hasGamepiece){
         //the beam brake is broken so we have something in the claw
         state = IntakeState.GETTING_OBJECT;
       } else {
         //we don't have shit and we just spinning the wheels.
         state = IntakeState.INTAKING;
       }
     } else if (ejecting) {
       //check if we're ejecting a cone or a cube:
       if(state == IntakeState.HAS_CONE) {
         state = IntakeState.EJECTING_CONE;
         RobotContainer.photonicHRI.runElement(ejectCone);
       }
       else if(state == IntakeState.HAS_CUBE) {
         state = IntakeState.EJECTING_CUBE;
         RobotContainer.photonicHRI.runElement(ejectCube);
       }
       else if(!ejecting()) {
         state = IntakeState.EJECTING_UNKNOWN; //we haven't detected a cone or a cube yet
         RobotContainer.photonicHRI.runElement(ejectUnknown);
       }

       //start the timer once the object is out of the beam brake:
       if(!hasGamepiece) ejectTimer.start();
       if(ejectTimer.get() > TIME_TO_EJECT){
         state = IntakeState.EJECTING_NOTHING;
         RobotContainer.photonicHRI.runElement(ejectNothing);
       }
     } else {
       var running = RobotContainer.photonicHRI.running;
       if(running != null){
         if(running == ejectCone || running == ejectCube || running == ejectUnknown || running == ejectNothing){
            RobotContainer.photonicHRI.off();
         }
        }
      if(hasGamepiece && color.confidence > 0.47) {
        //the color sensor sees something, so we now have sopmething:
        if(color.color.equals(CONE_COLOR)) state = IntakeState.HAS_CONE;
        else state = IntakeState.HAS_CUBE;
      }
       //we aren't spinning the wheels
       if(state != IntakeState.HAS_CONE && state != IntakeState.HAS_CUBE){
         //we're just chillin:
         state = IntakeState.RESTING;
        //  RobotContainer.photonicHRI.off();

       }
     }
     if(!ejecting){
       ejectTimer.stop();
       ejectTimer.reset();
     }
  }

  public Command runIntake(){
    return new RunCommand(() -> {
      setPercent(-1);
      setClaw(true);
    }, this);
  }

  public Command runIntakeClosed(){
    return new RunCommand(() -> {
      setPercent(-1);
      setClaw(false);
    }, this);
  }

  public Command runOuttake(){
    return new RunCommand(() -> {
      setPercent(1);
      setClaw(true);
    }, this);
  }

  public Command runOuttakeClosed(){
    return new RunCommand(() -> {
      setPercent(1);
      setClaw(false);
    }, this);
  }

  public Command drop(){
    return new RunCommand(() -> {
      setPercent(0);
      setClaw(true);
    }, this);
  }

  public Command rest(){
    return new RunCommand(() -> {
      setPercent(-0.03);
      setClaw(false);
    }, this);
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
    if(open == clawSolenoid.get()) {
      clawTimer.reset();
      clawTimer.start();
    }
    clawSolenoid.set(!open);
  }

  public void toggleClaw(){
    setClaw(clawSolenoid.get());
  }

  public boolean isClawOpen(){
    return !clawSolenoid.get() && clawTimer.get() > TIME_FOR_CLAW_TO_OPEN;
  }

  public boolean isClawClosed(){
     return clawSolenoid.get() && clawTimer.get() > TIME_FOR_CLAW_TO_OPEN;
  }

  public boolean ejecting(){
    return state == IntakeState.EJECTING_CONE || state == IntakeState.EJECTING_CUBE || state == IntakeState.EJECTING_NOTHING || state == IntakeState.EJECTING_UNKNOWN;
  }

  public boolean hasCube(){
    return state == IntakeState.HAS_CUBE || state == IntakeState.EJECTING_CUBE;
  }

  public boolean hasCone(){
    return state == IntakeState.HAS_CONE || state == IntakeState.EJECTING_CONE;
  }

  public IntakeState getState() {
    return state;
  }

   public Command grabCommand(boolean open){
     return new InstantCommand(()  -> setClaw(open), this);
   }

   public Command ejectCommand(){
        return new InstantCommand(() -> setPercent(1), this);
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
