/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.sensors;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Add your docs here.
 */
public class Pigeon {
    private final PigeonIMU pg;

    // private boolean bumped = false;

    public Pigeon(PigeonIMU pigeon){
        pg = pigeon;
    }

    public short[] getAcceleration(){
        var acceleration = new short[3];
        pg.getBiasedAccelerometer(acceleration);
        return acceleration;
    }
    /**
     * get the robots yaw from the pigeon.
     * @return the yaw in degrees
     */
    public double getYaw(){
        return pg.getYaw();
    }
    /**
     * get the robots pitch from the pigeon.
     * @return the pitch in degrees
     */
    public double getPitch(){
        return pg.getPitch();
    }
    /**
     * get the robots roll from the pigeon.
     * @return the roll in degrees
     */
    public double getRoll(){
        return pg.getRoll();
    }

    /**
     * get the fused accelerometer and magnetometer heading from the pigeon
     * @return the heading in degrees
     */
    public double getHeading(){
        return Math.IEEEremainder(pg.getFusedHeading(), 360.0d);
    }

    public Rotation2d getRotation(){
        return Rotation2d.fromDegrees(getHeading());
    }

    public void zeroHeading(){
        pg.setFusedHeading(0);
    }

    public void getRawGyro(double[] xyz){
        pg.getRawGyro(xyz);
    }
}
