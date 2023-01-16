// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    public Arm() {
    }

    /**
     * set the lift motors as a percentage (-1 to 1)
     * of full power
     * @param speed the percentage, positive drives out
     */
    public void setPercent(double speed){
        //todo
    }

    /**
     * get the distance from the pivot to the end of the
     * lift
     * @return lift extension meters
     */
    public double getExtension(){
        return 0; //todo
    }

    /**
     * set the lift extension in meters between
     * the pivot and the wrist
     */
    public void setExtension(double meters){
        //todo
    }

    public boolean finishedMotion(){
        return false; //todo
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
