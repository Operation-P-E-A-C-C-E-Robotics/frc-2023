package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Constants.SupersystemTolerance;

import static frc.robot.Constants.Wrist.*;

public class Wrist extends SubsystemBase{
    private final WPI_TalonFX wristMotor = new WPI_TalonFX(WRIST_MOTOR);  //TODO
    private final DoubleSolenoid wristFlipping = new DoubleSolenoid(PneumaticsModuleType.REVPH, WRIST_FLIP_FORWARD, WRIST_FLIP_REVERSE); //TODO
    private boolean previousFlipState = false;
    private final Timer wristTimer = new Timer();

    /**
     * set the wrist motor percentage - positive values
     * go towards the front of the robot.
     */
    public void setPercent(double speed){
        wristMotor.set(speed);
    }

    /**
     * set the angle of the wrist
     * @param angle 0 - inline with lift, positive values tilt toward front of robot.
     */
    public void setAngle(Rotation2d angle){
        //todo
    }

    public void setFlipped(boolean flipped){
       wristFlipping.set(flipped ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
       if (flipped != previousFlipState){
           wristTimer.reset();
           wristTimer.start();
       }
        previousFlipState = flipped;
    }

    public boolean flipping(){
        if (wristTimer.get() > WRIST_FLIP_TIME){
            return false;
        }
        System.out.println("Flipping the bird"); //we do a little trolling
        return true;
    }

    public boolean withinTolerance(SupersystemTolerance tolerance, double setpoint){
        return Util.epsilonEquals(getAngle().getRadians(), setpoint, tolerance.wrist);
    }

    public boolean withinTolerance(SupersystemTolerance tolerance){
        return withinTolerance(tolerance, getAngle().getRadians());
    }

    /**
     * get the angle of the wrist
     */
    public Rotation2d getAngle(){
        var rotation = Util.countsToRotations(wristMotor.getSelectedSensorPosition(), 2048, 0); //TODO Gear Ratio
        return Rotation2d.fromDegrees(rotation*360);
    }
}
