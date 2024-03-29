// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.safety.DankPids;
import frc.lib.util.ServoMotor;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Constants.SupersystemTolerance;
import frc.robot.DashboardManager;
import frc.robot.RobotState;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static frc.robot.Constants.Arm.*;

public class Arm extends SubsystemBase {
    private final ServoMotor servoController = new ServoMotor(SYSTEM_CONSTANTS, this::setVoltage, this::getExtension, this::getVelocity);
    private final WPI_TalonFX armMaster = new WPI_TalonFX(MASTER_PORT);
    private double setpoint = 0;
    private Supplier<Boolean> softLimit;
    private static final double SOFT_LIMIT = 1.13;
    private boolean needsToRectractToSoftLimit = false;

    /** Creates a new ExampleSubsystem. */
    public Arm() {
        armMaster.configFactoryDefault();
        armMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 50, 60, 0.1));

        armMaster.setInverted(Constants.Inversions.ARM);
        armMaster.configStatorCurrentLimit(CURRENT_LIMIT);
        armMaster.configLimitSwitchDisableNeutralOnLOS(true, 0);
        servoController.setPeriodicFunction(this::zeroFromLimitSwitchPeriodic);
        DankPids.registerDankTalon(armMaster);
    }

    public void setSoftLimit(Supplier<Boolean> tripped){
        this.softLimit = tripped;
    }

    /**
     * set the lift motors as a percentage (-1 to 1)
     * of full power
     * @param speed the percentage, positive drives out
     */
    public void setPercent(double speed){
        servoController.disableLoop();
        if(softLimit.get()) {
            armMaster.set(-0.4);
            return;
        }
        armMaster.set(speed);
    }

    public void setVoltage(double voltage){
        if(softLimit.get()) {
            armMaster.setVoltage(-4);
            return;
        }
        armMaster.setVoltage(voltage);
    }

    /**
     * get the distance from the pivot to the end of the
     * lift
     * @return lift extension meters
     */
    public double getExtension(){
        // return Util.countsToRotations(armMaster.getSelectedSensorPosition(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing);
        var counts = armMaster.getSelectedSensorPosition();
        var percentOfMax = counts / FULLY_EXTENDED_COUNTS;
        var extensionFromMin = percentOfMax * (MAX_EXTENSION - MIN_EXTENSION);
        return MIN_EXTENSION + extensionFromMin;
    }

    public double getVelocity(){
        return ((armMaster.getSelectedSensorVelocity()*10) / SYSTEM_CONSTANTS.cpr) / SYSTEM_CONSTANTS.gearing;
    }
    /**
     * set the lift extension in meters between
     * the pivot and the wrist
     */
    public void setExtension(double meters){
        setpoint = meters;
        servoController.enableLoop();
        servoController.goToState(meters);
    }

    public boolean withinTolerance(SupersystemTolerance tolerance, double setpoint){
        return Util.epsilonEquals(getExtension(), setpoint, tolerance.arm);
    }

    public boolean withinTolerance(SupersystemTolerance tolerance){
        return withinTolerance(tolerance, setpoint);
    }

    public double getEncoderCounts(){
        return armMaster.getSelectedSensorPosition();
    }

    public void zeroFromLimitSwitchPeriodic(){
        if(armMaster.isFwdLimitSwitchClosed() == 1){
            armMaster.setSelectedSensorPosition(0);
        }
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
        armSim.setInputVoltage(armMaster.get() * RobotController.getBatteryVoltage());
        armSim.update(0.02);
        armMotorSim.setIntegratedSensorRawPosition((int) Util.rotationsToCounts(armSim.getPositionMeters(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing));
        armMotorSim.setIntegratedSensorVelocity((int) Util.rotationsToCounts(armSim.getVelocityMetersPerSecond(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing));

        DashboardManager.getInstance().drawArmSim(armSim.getPositionMeters() * 150);
    }
}
