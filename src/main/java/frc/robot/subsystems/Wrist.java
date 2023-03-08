package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.safety.DankPids;
import frc.lib.util.ServoMotor;
import frc.lib.util.Util;
import frc.robot.Constants.SupersystemTolerance;
import frc.robot.Constants;
import frc.robot.DashboardManager;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Wrist.*;

public class Wrist extends ServoMotor {
    private final WPI_TalonFX wristMaster = new WPI_TalonFX(WRIST_MOTOR);  //TODO
    private final Solenoid wristSolenoid = new Solenoid(Constants.UPPER_PNEUMATICS_MODULE_CAN_ID, PneumaticsModuleType.CTREPCM, WRIST_FLIP_SOLENOID); //TODO
    private final DoubleSupplier pivotAngle;
    private boolean previousFlipState = false;
    private final Timer wristTimer = new Timer();

    public Wrist(DoubleSupplier pivotAngle){
        super(SYSTEM_CONSTANTS);
        wristMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 50, 60, 10));

        wristMaster.setInverted(false);
        wristMaster.configStatorCurrentLimit(CURRENT_LIMIT);

        this.pivotAngle = pivotAngle;

        if(Robot.isSimulation()) SmartDashboard.putNumber("wrist setpoint", 0);
        DankPids.registerDankTalon(wristMaster);

        setPeriodicFunction(this::flipWristPeriodic);

        wristTimer.start();
    }

    /**
     * set the wrist motor percentage - positive values
     * go towards the front of the robot.
     */
    public void setPercent(double speed){
        wristMaster.set(speed);
    }

    /**
     * set motor voltage
     * @param voltage positive values go towards the front of the robot.
     */
    public void setVoltage(double voltage){
        wristMaster.setVoltage(voltage);
    }

    /**
     * set the angle of the wrist
     * @param angle 0 - inline with lift, positive values tilt toward front of robot.
     */
    public void setAngle(Rotation2d angle){
        enableLoop(this::setVoltage, this::getAngleRads, this::getVelocityRads);
        goToState(angle.getRadians());
    }

    public void setFlipped(boolean flipped){
       wristSolenoid.set(flipped);
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
        return Util.epsilonEquals(getAngle().getRadians(), setpoint, tolerance.wrist) && !flipping();
    }

    public boolean withinTolerance(SupersystemTolerance tolerance){
        return withinTolerance(tolerance, getAngle().getRadians());
    }

    /**
     * get the angle of the wrist
     */
    public Rotation2d getAngle(){
        var rotation = Util.countsToRotations(wristMaster.getSelectedSensorPosition(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing); //TODO Gear Ratio
        return new Rotation2d(Units.rotationsToRadians(rotation));
    }

    public Rotation2d getVelocity(){
        var rotation = Util.countsToRotations(wristMaster.getSelectedSensorVelocity(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing); //TODO Gear Ratio
        return new Rotation2d(Units.rotationsToRadians(rotation));
    }

    public double getAngleRads(){
        return getAngle().getRadians();
    }

    public double getVelocityRads(){
        return getVelocity().getRadians();
    }

    public void flipWristPeriodic(){
        setFlipped(pivotAngle.getAsDouble() > 0);
    }

    //SIMULATION:
    private final SingleJointedArmSim wristSim = new SingleJointedArmSim(
            SYSTEM_CONSTANTS.motor,
            SYSTEM_CONSTANTS.gearing,
            SYSTEM_CONSTANTS.inertia,
            LENGTH,
            -100,
            100,
            MASS,
            false
    );
    private final TalonFXSimCollection wristMotorSim = wristMaster.getSimCollection();
    double prevSetpoint = 0;

    @Override
    public void simulationPeriodic(){
        //update from the dashboard setpoint:
        var setpoint = SmartDashboard.getNumber("wrist setpoint", 0);
        if (setpoint != prevSetpoint){
            setAngle(new Rotation2d(setpoint));
            prevSetpoint = setpoint;
        }
        SmartDashboard.putNumber("wrist angle", getAngle().getDegrees());
        SmartDashboard.putBoolean("wrist flipped", wristSolenoid.get());
        SmartDashboard.putBoolean("wrist flipping", flipping());
        DashboardManager.getInstance().drawWristSim(getAngle().getDegrees());

        wristSim.setInputVoltage(wristMaster.get() * RobotController.getBatteryVoltage());
        wristSim.update(0.02);

        wristMotorSim.setIntegratedSensorRawPosition(
                (int) Util.rotationsToCounts(
                        Units.radiansToRotations(wristSim.getAngleRads()
                                + (Math.PI / 2)
                                - (pivotAngle.getAsDouble() - Math.PI)),
                        SYSTEM_CONSTANTS.cpr,
                        SYSTEM_CONSTANTS.gearing
                )
        );
        wristMotorSim.setIntegratedSensorVelocity(
                (int) Util.rotationsToCounts(
                        Units.radiansToRotations(wristSim.getVelocityRadPerSec()),
                        SYSTEM_CONSTANTS.cpr,
                        SYSTEM_CONSTANTS.gearing
                )
        );
    }
}
