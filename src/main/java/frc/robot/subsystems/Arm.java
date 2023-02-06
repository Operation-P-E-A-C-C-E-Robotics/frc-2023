// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Constants.SupersystemTolerance;

import static frc.robot.Constants.Arm.*;

public class Arm extends SubsystemBase {
    private final WPI_TalonFX armMaster = new WPI_TalonFX(MASTER_PORT); //todo port number
    private double setpoint = 0;

    // private final WPI_TalonFX armSlave = new WPI_TalonFX(ARM_SLAVE); //todo do we need a slave?
    /** Creates a new ExampleSubsystem. */
    public Arm() {
    }

    /**
     * set the lift motors as a percentage (-1 to 1)
     * of full power
     * @param speed the percentage, positive drives out
     */
    public void setPercent(double speed){
        armMaster.set(speed);
    }

    /**
     * get the distance from the pivot to the end of the
     * lift
     * @return lift extension meters
     */
    public double getExtension(){
        return Util.countsToRotations(armMaster.getSelectedSensorPosition(), 2048, 0);
    }

    /**
     * set the lift extension in meters between
     * the pivot and the wrist
     */
    public void setExtension(double meters){
        setpoint = meters;
        //todo
    }

    public boolean withinTolerance(SupersystemTolerance tolerance, double setpoint){
        return Util.epsilonEquals(getExtension(), setpoint, tolerance.arm);
    }

    public boolean withinTolerance(SupersystemTolerance tolerance){
        return withinTolerance(tolerance, setpoint);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
