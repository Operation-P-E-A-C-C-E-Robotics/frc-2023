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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.trajectory.RealTimeTrapezoidalMotion;
import frc.lib.trajectory.RealTimeTrapezoidalMotionDraft;

import java.util.ArrayList;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class DCMotorSystemBase extends SubsystemBase {
    private final LinearSystem<N2, N1, N2> plant;
    private final LinearSystemLoop<N2, N1, N2> loop;
    private final ArrayList<Feedforward> feedforwards = new ArrayList<>();

    private final SystemConstants constants;

//    private TrapezoidProfile profile = new TrapezoidProfile(
//            new TrapezoidProfile.Constraints(0, 0),
//            new TrapezoidProfile.State(0, 0)
//    );
    private RealTimeTrapezoidalMotion testProfile;
    private final Timer profileTimer = new Timer();
    private boolean followingProfile = false, looping = false;

    private DoubleConsumer voltDriveFunction;
    private DoubleSupplier getPosition, getVelocity;

    private Runnable superPeriodic;

    /**
     * A messy ass helper class to run trajectories on a DC Motor state space controller.
     * @param constants The constants for the system
     */
    public DCMotorSystemBase(SystemConstants constants) {
        this.constants = constants;
        plant = LinearSystemId.createDCMotorSystem(
                constants.motor,
                constants.inertia,
                constants.gearing
        );
        KalmanFilter<N2, N1, N2> observer = new KalmanFilter<>(
                Nat.N2(),
                Nat.N2(),
                plant,
                VecBuilder.fill(constants.kalmanModelAccuracyPosition, constants.kalmanModelAccuracyVelocity),
                VecBuilder.fill(constants.kalmanSensorAccuracyPosition, constants.kalmanSensorAccuracyVelocity),
                constants.dt
        );
        LinearQuadraticRegulator<N2, N1, N2> linearQuadraticRegulator = new LinearQuadraticRegulator<>(
                plant,
                VecBuilder.fill(constants.lqrPositionTolerance, constants.lqrVelocityTolerance),
                VecBuilder.fill(constants.lqrControlEffortTolerance),
                constants.dt
        );
        //linearQuadraticRegulator.latencyCompensate(plant, constants.dt, constants.lqrSensorDelay);
        loop = new LinearSystemLoop<>(
                plant,
                linearQuadraticRegulator,
                observer,
                constants.maxVoltage,
                constants.dt
        );
        testProfile = new RealTimeTrapezoidalMotion(
                constants.maxVelocity, constants.maxAcceleration
        );
    }

    public void addFeedforward(Feedforward feedforward){
        feedforwards.add(feedforward);
    }

    public LinearSystem<N2, N1, N2> getSystem(){
        return plant;
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

    /**
     * @return whether the feedback loop is enabled
     */
    public boolean isLooping() {
        return looping;
    }

    /**
     * set a new trajectory to follow
     * @param profile The profile to follow
     */
    public void setTrajectory(RealTimeTrapezoidalMotion profile) {
        profileTimer.reset();
        this.testProfile = profile;
        followingProfile = true;
    }

    /**
     * stop following the current trajectory, but continue holding
     * the current reference
     */
    public void stopTrajectory() {
        followingProfile = false;
    }

    /**
     * generate a trajectory from the current position to the given position,
     * and start following it
     * @param position position setpoint
     * @param velocity velocity setpoint
     */
    public void goToState(double position, double velocity) {
        if(!looping){
            throw new IllegalStateException("Cannot set state without enabling loop");
        }
        var profile = new TrapezoidProfile(
                new TrapezoidProfile.Constraints(constants.maxVelocity, constants.maxAcceleration),
                new TrapezoidProfile.State(position, velocity),
                new TrapezoidProfile.State(getPosition.getAsDouble(), getVelocity.getAsDouble())
        );
        testProfile.setGoalState(position, velocity);
//        setTrajectory(profile);
    }

    /**
     * set the next reference for the feedback loop -
     * you probably don't want to use this, use goToState instead.
     * This set's the reference for the next iteration of the feedback loop directly,
     * and does not generate a trajectory.
     * @param position position reference
     * @param velocity velocity reference
     */
    public void setNextR(double position, double velocity) {
        loop.setNextR(VecBuilder.fill(position, velocity));
    }

    /**
     * allow the subsystem to run code in the periodic method
     * @param superPeriodic a function to run every periodic loop
     */
    public void setPeriodicFunction(Runnable superPeriodic) {
        this.superPeriodic = superPeriodic;
    }

    @Override
    public final void periodic() {
        // run the subsystem's periodic code
        if(superPeriodic != null) superPeriodic.run();
        if(!looping) return; // don't run the feedback loop if it's not enabled
        var time = profileTimer.get(); // get the time since the profile started

        // start the profile timer if it's not already running, and stop it if it's done
        if(followingProfile && time == 0){
            profileTimer.start();
        } else if(!followingProfile && time != 0){
            profileTimer.stop();
            profileTimer.reset();
        }

        var feedforward = 0.0;
        testProfile.setState(new TrapezoidProfile.State(getPosition.getAsDouble(), getVelocity.getAsDouble()));
        // if we're following a profile, calculate the next reference
        if(followingProfile){
            var output = testProfile.calculate(0.02);
            setNextR(output.position, output.velocity);
            for (var i : feedforwards) {
                feedforward += i.calculate(output.position, output.velocity);
            }
        }
        SmartDashboard.putNumber("Feedforward", feedforward);

        // run the feedback
        loop.correct(VecBuilder.fill(getPosition.getAsDouble(), getVelocity.getAsDouble()));
        loop.predict(constants.dt);
        voltDriveFunction.accept(loop.getU(0) + feedforward);
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
                lqrSensorDelay,
                maxVoltage,
                dt;
        public final DCMotor motor;

        /**
         * @param motor The motor to use
         * @param inertia The inertia of the system
         * @param gearing The gearing of the system
         * @param cpr The counts per revolution of the encoder
         * @param maxVelocity The maximum angular velocity the system can attain
         * @param maxAcceleration The maximum angular acceleration the system can attain
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
                               double maxVelocity,
                               double maxAcceleration,
                               double kalmanModelAccuracyPosition,
                               double kalmanModelAccuracyVelocity,
                               double kalmanSensorAccuracyPosition,
                               double kalmanSensorAccuracyVelocity,
                               double lqrPositionTolerance,
                               double lqrVelocityTolerance,
                               double lqrControlEffortTolerance,
                               double lqrSensorDelay,
                               double maxVoltage,
                               double dt) {
            this.motor = motor;
            this.inertia = inertia;
            this.gearing = gearing;
            this.cpr = cpr;
            this.maxVelocity = maxVelocity;
            this.maxAcceleration = maxAcceleration;
            this.kalmanModelAccuracyPosition = kalmanModelAccuracyPosition;
            this.kalmanModelAccuracyVelocity = kalmanModelAccuracyVelocity;
            this.kalmanSensorAccuracyPosition = kalmanSensorAccuracyPosition;
            this.kalmanSensorAccuracyVelocity = kalmanSensorAccuracyVelocity;
            this.lqrPositionTolerance = lqrPositionTolerance;
            this.lqrVelocityTolerance = lqrVelocityTolerance;
            this.lqrControlEffortTolerance = lqrControlEffortTolerance;
            this.lqrSensorDelay = lqrSensorDelay;
            this.maxVoltage = maxVoltage;
            this.dt = dt;
        }
    }

    public static interface Feedforward{
        double calculate(double position, double velocity);
    }
}
