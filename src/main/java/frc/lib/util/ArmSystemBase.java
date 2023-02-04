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
import frc.lib.util.DCMotorSystemBase.SystemConstants;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ArmSystemBase extends SubsystemBase {
    private final LinearSystem<N2, N1, N1> plant;
    private final KalmanFilter<N2, N1, N1> observer;
    private final LinearQuadraticRegulator<N2, N1, N1> linearQuadraticRegulator;
    private final LinearSystemLoop<N2, N1, N1> loop;
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
    public ArmSystemBase(SystemConstants constants) {
        this.constants = constants;
        plant = LinearSystemId.createSingleJointedArmSystem(constants.motor, constants.inertia, constants.gearing);
        observer = new KalmanFilter<N2, N1, N1>(
                Nat.N2(),
                Nat.N1(),
                plant,
                VecBuilder.fill(constants.kalmanModelAccuracyPosition, constants.kalmanModelAccuracyVelocity),
                VecBuilder.fill(constants.kalmanSensorAccuracyPosition),
                constants.dt
        );
        linearQuadraticRegulator = new LinearQuadraticRegulator<N2, N1, N1>(
                plant,
                VecBuilder.fill(constants.lqrPositionTolerance, constants.lqrVelocityTolerance),
                VecBuilder.fill(constants.lqrControlEffortTolerance),
                constants.dt
        );
        loop = new LinearSystemLoop<N2, N1, N1>(
                plant,
                linearQuadraticRegulator,
                observer,
                constants.maxVoltage,
                constants.dt
        );
    }

    public LinearSystem<N2, N1, N1> getSystem(){
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
            SmartDashboard.putNumber("setpt position", output.position);
            SmartDashboard.putNumber("setpt velocity", output.velocity);
            setNextR(output.position, output.velocity);
        }

        SmartDashboard.putNumber("position", getPosition.getAsDouble());
        SmartDashboard.putNumber("velocity", getVelocity.getAsDouble());

        loop.correct(VecBuilder.fill(getPosition.getAsDouble()));
        loop.predict(constants.dt);
        voltDriveFunction.accept(loop.getU(0));
    }
}
