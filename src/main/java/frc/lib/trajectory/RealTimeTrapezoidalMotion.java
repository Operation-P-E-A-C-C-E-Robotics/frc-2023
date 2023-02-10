package frc.lib.trajectory;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.lib.util.Util;

public class RealTimeTrapezoidalMotion {

    private final double maxVelocity;
    private final double maxAcceleration;
    private State currentState = new State(0,0);
    private State goalState = new State(0,0);

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
    public RealTimeTrapezoidalMotion(double maxVelocity, double maxAcceleration){
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public State calculate(double dt){
        State constranedState;
        var decelerationPosition = calcDecelerationPosition(dt);
        if(currentState.position >= decelerationPosition){
            //we are in the deceleration phase
            constranedState = constrainDeceleration(currentState.position, decelerationPosition, dt);
        } else {
            //we are in the acceleration/coast phase
            constranedState = constrainAcceleration(currentState.position, goalState.position, dt);
        }
        return constranedState;
    }

    public void setGoalState(double position, double velocity){
        goalState = new State(position, velocity);
    }

    public State constrainAcceleration(double initialPosition, double finalPosition, double dt){
        var velocity = finalPosition - initialPosition;
        var acceleration = velocity - currentState.velocity;
        return constrain(initialPosition, velocity, acceleration, dt);
    }

    public State constrainDeceleration(double initialPosition, double finalPosition, double dt){
        return constrain(initialPosition, finalPosition - initialPosition, -maxAcceleration, dt);
    }

    public State constrain(double initialPosition, double targetVelocity, double targetAcceleration, double dt){
        // figure out which constraint is limiting the motion
        if(Math.abs(targetAcceleration) > maxAcceleration){
            // if acceleration is limiting, then we need to adjust the velocity to match the acceleration
            targetAcceleration = Util.limit(targetAcceleration, maxAcceleration);
            targetVelocity = integrate(currentState.velocity, targetAcceleration, dt);
        }
        if(Math.abs(targetVelocity) > maxVelocity){
            // if velocity is limiting, acceleration can be ignored (since acceleration isn't what we are
            // returning)
            targetVelocity = Util.limit(targetVelocity, maxVelocity);
        }
        // calculate the actual attainable position
        initialPosition = integrate(initialPosition, targetVelocity, dt);
        return new State(initialPosition, targetVelocity);
    }

    public double calcDecelerationPosition(double dt){
        System.out.println("calc distance to decelerate: " + calcDistanceToDecelerate(dt));
        return goalState.position - calcDistanceToDecelerate(dt);
    }

    public double calcTimeToDecelerate(){
        return currentState.velocity / maxAcceleration;
    }

    public double calcDistanceToDecelerate(double dt){
        return integrateSlope(0, currentState.velocity, calcTimeToDecelerate());
    }

    public void setState(State state){
        this.currentState = state;
    }

    //constant velocity P = V * t
    public double integrate(double initial, double value, double dt){
        return initial + (value * dt);
    }

    public double integrateSlope(double initial, double slope, double dt){
        return integrate(initial*2, (dt/slope) - initial, dt) / 2;
    }
}
