package frc.lib.util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.Turret.DT;

public class LQRMotorControllerBase extends SubsystemBase {
    private final LinearSystem<N2, N1, N2> plant;
    private final KalmanFilter<N2, N1, N2> observer;
    private final LinearQuadraticRegulator<N2, N1, N2> linearQuadraticRegulator;
    private final LinearSystemLoop<N2, N1, N2> loop;
    private final SystemConstants constants;
    private TrapezoidProfile profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(0, 0),
            new TrapezoidProfile.State(0, 0)
    );
    private boolean followingProfile = false, looping = false;
    private final Timer profileTimer = new Timer();
    private DoubleConsumer voltDriveFunction;
    private DoubleSupplier getPosition, getVelocity;
    private Runnable superPeriodic;

    /**
     * A messy ass helper class to run trajectories on a DC Motor state space controller.
     * @param constants The constants for the system
     */
    public LQRMotorControllerBase(SystemConstants constants) {
        this.constants = constants;
        plant = LinearSystemId.createDCMotorSystem(
                constants.motor,
                constants.inertia,
                constants.gearing
        );
        observer = new KalmanFilter<N2, N1, N2>(
                Nat.N2(),
                Nat.N2(),
                plant,
                VecBuilder.fill(constants.kalmanModelAccuracyPosition, constants.kalmanModelAccuracyVelocity),
                VecBuilder.fill(constants.kalmanSensorAccuracyPosition, constants.kalmanSensorAccuracyVelocity),
                constants.dt
        );
        linearQuadraticRegulator = new LinearQuadraticRegulator<N2, N1, N2>(
                plant,
                VecBuilder.fill(constants.lqrPositionTolerance, constants.lqrVelocityTolerance),
                VecBuilder.fill(constants.lqrControlEffortTolerance),
                constants.dt
        );
        loop = new LinearSystemLoop<N2, N1, N2>(
                plant,
                linearQuadraticRegulator,
                observer,
                constants.maxVoltage,
                constants.dt
        );
    }

    /**
     * enable the feedback loop with a consumer to set the voltage, and a supplier to get the position and velocity
     * @param voltDriveFunction The function to set the voltage
     * @param getPosition The function to get the position
     * @param getVelocity The function to get the velocity
     */
    protected void enableLoop(DoubleConsumer voltDriveFunction, DoubleSupplier getPosition, DoubleSupplier getVelocity) {
        this.voltDriveFunction = voltDriveFunction;
        this.getPosition = getPosition;
        this.getVelocity = getVelocity;
        looping = true;
    }

    /**
     * disable the feedback loop (and give control back to the subsystem)
     */
    public void disableLoop() {
        looping = false;
    }

    public boolean isLooping() {
        return looping;
    }

    /**
     * set a new trajectory to follow
     * @param profile The profile to follow
     */
    public void setTrajectory(TrapezoidProfile profile) {
        profileTimer.reset();
        this.profile = profile;
        followingProfile = true;
    }

    /**
     * stop following the current trajectory, but continue holding
     * the current reference
     */
    public void stopTrajectory() {
        followingProfile = false;
    }

    public void setTrajectory(TrapezoidProfile profile, double timeout) {
        profileTimer.reset();
        profileTimer.start();

        this.profile = profile;
        followingProfile = true;
    }

    public void goToState(double position, double velocity) {
        if(!looping){
            throw new IllegalStateException("Cannot set state without enabling loop");
        }
        var profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(constants.maxVelocity, constants.maxAcceleration),
                new TrapezoidProfile.State(position, velocity),
                new TrapezoidProfile.State(getPosition.getAsDouble(), getVelocity.getAsDouble())
        );
        setTrajectory(profile);
    }

    public void setNextR(double position, double velocity) {
        loop.setNextR(VecBuilder.fill(position, velocity));
    }

    public void setPeriodicFunction(Runnable superPeriodic) {
        this.superPeriodic = superPeriodic;
    }

    @Override
    public final void periodic() {
        if(superPeriodic != null) superPeriodic.run();
        if(!looping) return;
        var time = profileTimer.get();

        if(followingProfile && time == 0){
            profileTimer.start();
        } else if(!followingProfile && time != 0){
            profileTimer.stop();
            profileTimer.reset();
        }
        if(followingProfile){
            var output = profile.calculate(time);
            setNextR(output.position, output.velocity);
        }

        loop.correct(VecBuilder.fill(getPosition.getAsDouble(), getVelocity.getAsDouble()));
        loop.predict(DT);
        voltDriveFunction.accept(loop.getU(0));
    }

    public static final class SystemConstants {
        public final double inertia,
                gearing,
                cpr,
                maxVelocity,
                maxAcceleration,
                kalmanModelAccuracyPosition,
                kalmanModelAccuracyVelocity,
                kalmanSensorAccuracyPosition,
                kalmanSensorAccuracyVelocity,
                lqrPositionTolerance,
                lqrVelocityTolerance,
                lqrControlEffortTolerance,
                maxVoltage,
                dt;
        private final DCMotor motor;

        /**
         * @param motor The motor to use
         * @param inertia The inertia of the system
         * @param gearing The gearing of the system
         * @param cpr The counts per revolution of the encoder
         * @param maxAngularVelocity The maximum angular velocity the system can attain
         * @param maxAngularAcceleration The maximum angular acceleration the system can attain
         * @param kalmanModelAccuracyPosition The accuracy of the model for position
         * @param kalmanModelAccuracyVelocity The accuracy of the model for velocity
         * @param kalmanSensorAccuracyPosition The accuracy of the sensor for position
         * @param kalmanSensorAccuracyVelocity The accuracy of the sensor for velocity
         * @param lqrPositionTolerance How aggressively to correct for position error
         * @param lqrVelocityTolerance How aggressively to correct for velocity error
         * @param lqrControlEffortTolerance How aggressively to minimize control effort
         * @param maxVoltage The maximum voltage to try to attain
         * @param dt The time between each loop
         */
        public SystemConstants(DCMotor motor,
                               double inertia,
                               double gearing,
                               double cpr,
                               double maxAngularVelocity,
                               double maxAngularAcceleration,
                               double kalmanModelAccuracyPosition,
                               double kalmanModelAccuracyVelocity,
                               double kalmanSensorAccuracyPosition,
                               double kalmanSensorAccuracyVelocity,
                               double lqrPositionTolerance,
                               double lqrVelocityTolerance,
                               double lqrControlEffortTolerance,
                               double maxVoltage,
                               double dt) {
            this.motor = motor;
            this.inertia = inertia;
            this.gearing = gearing;
            this.cpr = cpr;
            this.maxVelocity = maxAngularVelocity;
            this.maxAcceleration = maxAngularAcceleration;
            this.kalmanModelAccuracyPosition = kalmanModelAccuracyPosition;
            this.kalmanModelAccuracyVelocity = kalmanModelAccuracyVelocity;
            this.kalmanSensorAccuracyPosition = kalmanSensorAccuracyPosition;
            this.kalmanSensorAccuracyVelocity = kalmanSensorAccuracyVelocity;
            this.lqrPositionTolerance = lqrPositionTolerance;
            this.lqrVelocityTolerance = lqrVelocityTolerance;
            this.lqrControlEffortTolerance = lqrControlEffortTolerance;
            this.maxVoltage = maxVoltage;
            this.dt = dt;
        }
    }
}
