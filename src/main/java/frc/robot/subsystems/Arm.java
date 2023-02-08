// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Constants.SupersystemTolerance;
import frc.robot.DashboardManager;

import static frc.robot.Constants.Arm.*;

public class Arm extends SubsystemBase {
    private final WPI_TalonFX armMaster = new WPI_TalonFX(MASTER_PORT); //todo port number
    private double setpoint = 0;

    // private final WPI_TalonFX armSlave = new WPI_TalonFX(ARM_SLAVE); //todo do we need a slave?
    /** Creates a new ExampleSubsystem. */
    public Arm() {
        SmartDashboard.putNumber("arm setpoint", 0);
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

    //SIMULATION:
    private final ElevatorSim armSim = new ElevatorSim(
            SYSTEM_CONSTANTS.motor,
            SYSTEM_CONSTANTS.gearing,
            CARRAIGE_MASS,
            1,
            MIN_EXTENSION,
            MAX_EXTENSION,
            false,
            VecBuilder.fill(0)
    );
    private final TalonFXSimCollection armMotorSim = armMaster.getSimCollection();
    double prevSetpoint = 0;
    @Override
    public void simulationPeriodic(){
        //update with setpoint from dashboard:
        setpoint = SmartDashboard.getNumber("arm setpoint", 0);
        if(setpoint != prevSetpoint){
            setPercent(setpoint);
            prevSetpoint = setpoint;
        }

        armSim.setInputVoltage(armMaster.get() * RobotController.getBatteryVoltage());
        armSim.update(0.02);
        armMotorSim.setIntegratedSensorRawPosition((int) Util.rotationsToCounts(armSim.getPositionMeters(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing));
        armMotorSim.setIntegratedSensorVelocity((int) Util.rotationsToCounts(armSim.getVelocityMetersPerSecond(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing));

        DashboardManager.getInstance().updateArmLength(armSim.getPositionMeters() * 150);
    }
}
