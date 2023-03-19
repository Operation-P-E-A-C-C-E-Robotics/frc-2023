package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.safety.DankPids;
import frc.lib.util.ServoMotor;
import frc.lib.util.Util;
import frc.robot.Constants.SupersystemTolerance;
import frc.robot.Constants;
import frc.robot.DashboardManager;
import frc.robot.Robot;

import java.util.function.DoubleSupplier;
import static frc.robot.Constants.Wrist.*;

public class Wrist extends SubsystemBase {
    private final ServoMotor servoController = new ServoMotor(SYSTEM_CONSTANTS, this::setVoltage, this::getAngleRads, this::getVelocityRads);

    private final WPI_TalonFX wristMaster = new WPI_TalonFX(WRIST_MOTOR);
    private final Solenoid wristSolenoid = new Solenoid(Constants.UPPER_PNEUMATICS_MODULE_CAN_ID, PneumaticsModuleType.CTREPCM, WRIST_FLIP_SOLENOID);
    private final DoubleSupplier pivotAngle;
    private boolean previousFlipState = false;
    private final Timer wristTimer = new Timer();
    private double setpoint = 0;

    public Wrist(DoubleSupplier pivotAngle){
        wristMaster.configFactoryDefault();

        wristMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 50, 60, 10));

        wristMaster.setInverted(Constants.Inversions.WRIST);
        wristMaster.configStatorCurrentLimit(CURRENT_LIMIT);

        wristMaster.configForwardSoftLimitEnable(true);
        wristMaster.configReverseSoftLimitEnable(true);
        wristMaster.configForwardSoftLimitThreshold(Util.rotationsToCounts(Units.degreesToRotations(90), SYSTEM_CONSTANTS));
        wristMaster.configReverseSoftLimitThreshold(Util.rotationsToCounts(Units.degreesToRotations(-90), SYSTEM_CONSTANTS));

        wristMaster.setNeutralMode(NeutralMode.Coast);

        wristMaster.setSelectedSensorPosition(Util.rotationsToCounts(0, SYSTEM_CONSTANTS));

        this.pivotAngle = pivotAngle;

        if(Robot.isSimulation()) SmartDashboard.putNumber("wrist setpoint", 0);
        DankPids.registerDankTalon(wristMaster);

        wristTimer.start();
    }

    /**
     * set the wrist motor percentage - positive values
     * go towards the front of the robot.
     */
    public void setPercent(double speed){
        servoController.disableLoop();
        wristMaster.set(speed);
    }

    /**
     * set motor voltage
     * @param voltage positive values go towards the front of the robot.
     */
    private void setVoltage(double voltage){
        wristMaster.setVoltage(voltage);
    }

    /**
     * set the angle of the wrist
     * @param angle 0 - inline with lift, positive values tilt toward front of robot.
     */
    public void setAngle(Rotation2d angle){
        servoController.enableLoop();
        setpoint = angle.getRadians();
        servoController.goToState(angle.getRadians() - pivotAngle.getAsDouble());
        // servoController.goToState(angle.getRadians());
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

    public void setNeutralMode(NeutralMode mode){
        wristMaster.setNeutralMode(mode);
    }

    public boolean withinTolerance(SupersystemTolerance tolerance, double setpoint){
        return Util.epsilonEquals(getAngle().getRadians(), setpoint, tolerance.wrist) && !flipping();
    }

    public boolean withinTolerance(SupersystemTolerance tolerance){
        return withinTolerance(tolerance, getAngle().getRadians());
    }

    public void zero(){
        wristMaster.setSelectedSensorPosition(0);
    }

    /**
     * get the angle of the wrist
     */
    public Rotation2d getAngle(){
        var rotation = Util.countsToRotations(wristMaster.getSelectedSensorPosition(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing);
        return new Rotation2d(Units.rotationsToRadians(rotation));
    }

    public Rotation2d getVelocity(){
        var rotation = Util.countsToRotations(wristMaster.getSelectedSensorVelocity(), SYSTEM_CONSTANTS.cpr, SYSTEM_CONSTANTS.gearing);
        return new Rotation2d(Units.rotationsToRadians(rotation));
    }

    public double getAngleRads(){
        return getAngle().getRadians();
    }

    public double getVelocityRads(){
        return getVelocity().getRadians();
    }

    @Override
    public void periodic(){
        setFlipped(pivotAngle.getAsDouble() > 0);
        if(servoController.isLooping()) servoController.goToState(setpoint - pivotAngle.getAsDouble());
    }

    //SIMULATION:
    private SingleJointedArmSim wristSim;
    private TalonFXSimCollection wristMotorSim;
    Double prevSetpoint = null;

    @Override
    public void simulationPeriodic(){
        if(wristSim == null) wristSim = new SingleJointedArmSim(
                SYSTEM_CONSTANTS.motor,
                SYSTEM_CONSTANTS.gearing,
                SYSTEM_CONSTANTS.inertia,
                LENGTH,
                -100,
                100,
                MASS,
                false
        );
        if(wristMotorSim == null) wristMotorSim = wristMaster.getSimCollection();
        if(prevSetpoint == null) prevSetpoint = 0.0;
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
