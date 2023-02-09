package frc.lib.util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.DCMotorSystemBase.SystemConstants;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

public class ArmSystemBase extends SubsystemBase {
    private final LinearSystem<N2, N1, N1> plant;
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
    private final double armLength, armMass;
    /**
     * A messy ass helper class to run trajectories on an arm
     * @param constants The constants for the system
     * @param armLength The length of the arm (m)
     * @param armMass The mass of the arm (kg)
     */
    public ArmSystemBase(SystemConstants constants, double armLength, double armMass) {
        this.constants = constants;
        plant = LinearSystemId.createSingleJointedArmSystem(constants.motor, constants.inertia, constants.gearing);
        KalmanFilter<N2, N1, N1> observer = new KalmanFilter<N2, N1, N1>(
                Nat.N2(),
                Nat.N1(),
                plant,
                VecBuilder.fill(constants.kalmanModelAccuracyPosition, constants.kalmanModelAccuracyVelocity),
                VecBuilder.fill(constants.kalmanSensorAccuracyPosition),
                constants.dt
        );
        LinearQuadraticRegulator<N2, N1, N1> linearQuadraticRegulator = new LinearQuadraticRegulator<N2, N1, N1>(
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
        this.armLength = armLength;
        this.armMass = armMass;
        SmartDashboard.putNumber("Arm Gravity Feedforward Multiplier", 1);
    }

    /**
     * get the LinearSystem being used by the loop
     * @return the LinearSystem
     */
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

    /**
     * tell whether the feedback loop is enabled (and thus whether the subsystem has control)
     * @return whether the loop is enabled
     */
    public boolean isLooping() {
        return looping;
    }

    /**
     * calculate additional feedforward to account for gravity
     */
    public double calculateGravityFeedforward(double position, double velocity){
        var force = armMass     * armLength
                                * 9.8
                                * 3.0
                                * constants.inertia
                                / (armMass * armLength * armLength)
                                * Math.cos(position - Math.PI*1.5);
        SmartDashboard.putNumber("Arm Gravity Force before gearbox", force);
        //account for gearing:
        force /= constants.gearing;

        SmartDashboard.putNumber("Arm Gravity Force after gearbox", force);
        //calculate voltage needed to counteract force:
        return constants.motor.getVoltage(force, velocity);
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

    /**
     * generate and follow a new trajectory from the current state to the given state
     * @param position The position to go to
     * @param velocity The velocity to go to
     */
    public void goToState(double position, double velocity) {
        //this.positionSetpoint = position; //TODO for testing only
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

    /**
     * set the next reference for the feedback loop
     * @param position The position reference
     * @param velocity The velocity reference
     */
    public void setNextR(double position, double velocity) {
        loop.setNextR(VecBuilder.fill(position, velocity));
    }

    /**
     * allow the subsystem to run a periodic function
     * @param superPeriodic The function to run - will run before the feedback loop every periodic call
     */
    public void setPeriodicFunction(Runnable superPeriodic) {
        this.superPeriodic = superPeriodic;
    }

    @Override
    public final void periodic() {
        if(superPeriodic != null) superPeriodic.run();
        if(!looping) return; // don't run the loop if it's not enabled

        var time = profileTimer.get();
        double feedforward = 0;

        // make sure the timer is running if we're following a profile, reset it if we're not
        if(followingProfile && time == 0){
            profileTimer.start();
        } else if(!followingProfile && time != 0){
            profileTimer.stop();

            profileTimer.reset();
        }

        // if we're following a profile, calculate the next reference and feedforward
        if(followingProfile){
            var output = profile.calculate(time);
            setNextR(output.position, output.velocity);
            feedforward = calculateGravityFeedforward(output.position, output.velocity);
        } else {
            feedforward = calculateGravityFeedforward(getPosition.getAsDouble(), getVelocity.getAsDouble());
        }
        SmartDashboard.putNumber("arm feedforward", feedforward);

        // run the feedback.
        loop.correct(VecBuilder.fill(getPosition.getAsDouble()));
        loop.predict(constants.dt);
        voltDriveFunction.accept(loop.getU(0) + feedforward);
        //feedforward only (for testing):
//        voltDriveFunction.accept(feedforward);
    }

    public static void main(String args[]){
        var armMass = 1;
        var armLength = 1;
        var gearing = 1;
        var motor = DCMotor.getFalcon500(2);

        //test the feedforward
        for(var i = 0; i < 360; i++){
            var angle = Math.toRadians(i) + Math.PI*1.5;
            var force = (armMass * 9.80665) * armLength * Math.cos(angle + Math.PI);
            force *= 1/gearing;
            var voltage = motor.getVoltage(force, 0);
            System.out.println(i + "," + voltage);
        }
    }
}
