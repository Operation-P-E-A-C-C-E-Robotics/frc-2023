package frc.lib.trajectory;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.lib.util.Util;

public class TrapezoidalMotion {

    private final double maxVelocity;
    private final double maxAcceleration;
    private State currentState = new State(0,0);
    private State goalState = new State(0,0);
    private State deltaState = new State(0,0);

    /**
     * This class is a better version of the TrapezoidalMotion class. It generates trapezoidal
     * motion profiles on the fly, rather than pre-generating them. This allows for more accurate
     * motion profiles, and allows for the robot to be able to change its motion profile in real-time.
     * How it works:
     * <br/>1. Calculate the position at which the robot will start decelerating. Based on this, we know if we are in the acceleration phase or the deceleration phase.
     * <br/><br/>2. If we are in acceleration/coast, calculate the total acceleration and velocity required to reach the goal position.
     *      then, constrain that acceleration and velocity to the max acceleration and velocity, and calculate the attainable position.
     * <br/><br/>3. If we are in deceleration, assume maximum deceleration, and calculate the attainable position.
     */
    public TrapezoidalMotion(double maxVelocity, double maxAcceleration){
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public void setCurrentState(double position, double velocity){
        currentState = new State(position, velocity);
    }

    public void setGoalState(double position, double velocity){
        goalState = new State(position, velocity);

    }

    public State calculate(double dt){
        // calculate the motions for deceleration (current velocity to the goal position with maximum acceleration),
        // and acceleration (current velocity to goal velocity with maximum acceleration)
        boolean inverted = goalState.position < currentState.position;
        double velocityToReachGoal = (Math.abs(goalState.position - currentState.position) * (inverted ? -1 : 1)) / dt;
        double accelerationVelocity = Util.limit(velocityToReachGoal, maxVelocity);
        var deceleration = Motion.fromPosition(currentState.velocity, goalState.velocity, velocityToReachGoal);
//        deceleration = Motion.fromState(new State(currentState.position, currentState.velocity), new State(goalState.position, goalState.velocity));
        var acceleration = Motion.fromTime(currentState.velocity, accelerationVelocity, dt);//.limitAccelerationConstantTime(maxAcceleration);
//        System.out.println("Accel: " + acceleration);
//        System.out.println("Decel: " + deceleration);
//        System.out.println("inverted: " + inverted);
//        System.out.println("velocityToReachGoal: " + velocityToReachGoal);
//        System.out.println("accelerationVelocity: " + accelerationVelocity);
        double position = currentState.position + acceleration.deltaPosition, velocity;
        boolean needsToDecelerate = Math.abs(deceleration.acceleration) >= maxAcceleration && !Double.isInfinite(deceleration.acceleration);
        if(needsToDecelerate){
            // we are in the deceleration phase
            System.out.println("Decelerating");
            deceleration = deceleration.interpolateTimeDeceleration(dt);
            position = currentState.position + (deceleration.deltaPosition);
            velocity = deceleration.finalVelocity;
        } else {
            System.out.println("Accelerating");
//            position = currentState.position + acceleration.deltaPosition;
            velocity = acceleration.finalVelocity;
        }
        return new State(position, velocity);
    }

    public static void main(String[] args){
        var test = new TrapezoidalMotion(2, 0.1);
        test.setCurrentState(-0.5, -0.1);
//        test.setGoalState(-0.5, 0);
            test.setGoalState(0.5, 0);
        for(int i = 0; i < 1000; i++){
            System.out.println(test.currentState.velocity);
            var state = test.calculate(0.5);
            test.setCurrentState(-state.position, -state.velocity);
        }
    }
}
