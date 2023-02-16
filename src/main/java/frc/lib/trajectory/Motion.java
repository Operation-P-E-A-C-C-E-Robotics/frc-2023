package frc.lib.trajectory;

import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.lib.util.Util;

import javax.sound.sampled.Line;

public class Motion {
    public final double deltaPosition, initialVelocity, finalVelocity, acceleration, time;
    public final boolean inverted;

    /**
     * take an initial state (velocity, position) and a final state (velocity, position),
     * and interpolate between them, accelerating at a constant rate
     * This interpolation includes position, velocity, and acceleration at each point in time.
     */
    private Motion(double deltaPosition, double initialVelocity, double finalVelocity, double acceleration, double time){
        this.deltaPosition = deltaPosition;
        this.initialVelocity = initialVelocity;
        this.finalVelocity = finalVelocity;
        this.acceleration = acceleration;
        this.time = time;
        inverted = false;
    }

    private Motion(double deltaPosition, double initialVelocity, double finalVelocity, double acceleration, double time, boolean inverted){
        this.deltaPosition = deltaPosition;
        this.initialVelocity = initialVelocity;
        this.finalVelocity = finalVelocity;
        this.acceleration = acceleration;
        this.time = time;
        this.inverted = inverted;
    }

    public Motion interpolateTime(double time){
//        if(inverted) return interpolateTimeDeceleration(time);
        var finalVelocity = Util.interpolate(initialVelocity, this.finalVelocity, time / this.time);
        return fromTime(initialVelocity, finalVelocity, time);
    }

    public Motion interpolateTimeDeceleration(double time){
        var finalVelocity = Util.interpolate(initialVelocity, this.finalVelocity, (this.time - time) / this.time);
        return fromTime(initialVelocity, finalVelocity, this.time - time);
    }
    public Motion limitVelocityConstantTime(double maxVelocity){
        return fromTime(initialVelocity, Util.limit(finalVelocity, maxVelocity), time);
    }

    public Motion limitVelocityConstantPosition(double maxVelocity){
        return fromPosition(initialVelocity, Util.limit(finalVelocity, maxVelocity), deltaPosition);
    }

    public Motion limitAccelerationConstantTime(double maxAcceleration){
        return fromVelocityAcceleration(initialVelocity, Util.limit(acceleration, maxAcceleration), time);
    }

    public Motion limitAccelerationConstantPosition(double maxAcceleration){
        return fromPositionAcceleration(initialVelocity, Util.limit(acceleration, maxAcceleration), deltaPosition);
    }

    public static Motion fromState(State initialState, State finalState){
        var deltaPosition = finalState.position - initialState.position;
        var initialVelocity = initialState.velocity;
        var finalVelocity = finalState.velocity;
        var inverted = deltaPosition < 0;

        if(inverted){
//            deltaPosition = -deltaPosition;
//            initialVelocity = finalVelocity;
//            finalVelocity = initialVelocity;
        }

        var time = deltaPosition / (initialVelocity + (finalVelocity - initialVelocity) / 2);
        var acceleration = (finalVelocity - initialVelocity) / time;

        if(Double.isNaN(time)) time = 0;
        if(Double.isNaN(acceleration)) acceleration = 0;

        return new Motion(deltaPosition, initialVelocity, finalVelocity, acceleration, time, inverted);
    }

    public static Motion fromPosition(double initialVelocity, double finalVelocity, double deltaPosition){
        var inverted = deltaPosition < 0;
//        if(inverted){
//            deltaPosition = -deltaPosition;
//            initialVelocity = finalVelocity;
//            finalVelocity = initialVelocity;
//        }
        var time = deltaPosition / (initialVelocity + ((finalVelocity - initialVelocity) / 2));
        var acceleration = (finalVelocity - initialVelocity) / time;
        if(Double.isNaN(time)) time = 0;
        if(Double.isNaN(acceleration)) acceleration = 0;
        return new Motion(deltaPosition, initialVelocity, finalVelocity, acceleration, time, inverted);
    }

    public static Motion fromTime(double initialVelocity, double finalVelocity, double time){
        var deltaPosition = time * (initialVelocity + ((finalVelocity - initialVelocity) / 2));
        return fromPosition(initialVelocity, finalVelocity, deltaPosition);
    }

    public static Motion fromVelocityAcceleration(double initialVelocity, double acceleration, double time){
        var finalVelocity = initialVelocity + (acceleration * time);
        return fromTime(initialVelocity, finalVelocity, time);
    }

    public static Motion fromPositionAcceleration(double initialVelocity, double acceleration, double deltaPosition){
        var time = Math.sqrt((2 * deltaPosition) / acceleration);
        return fromVelocityAcceleration(initialVelocity, acceleration, time);
    }

    public String toString(){
        return "deltaPosition: " + deltaPosition + " initialVelocity: " + initialVelocity + " finalVelocity: " + finalVelocity + " acceleration: " + acceleration + " time: " + time;
    }

    public static void main(String[] args){
        var test = Motion.fromState(new State(0, 0), new State(1, 1));
        System.out.println(test);
        for(double i = 0; i < test.time; i += 0.1){
            System.out.println(test.interpolateTime(i));
        }
        for(double i = 0; i < test.time; i += 0.1){
            System.out.println(test.interpolateTimeDeceleration(i));
        }
    }
}
