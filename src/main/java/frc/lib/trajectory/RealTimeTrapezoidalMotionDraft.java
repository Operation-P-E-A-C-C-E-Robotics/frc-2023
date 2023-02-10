package frc.lib.trajectory;

import frc.lib.util.Util;

public class RealTimeTrapezoidalMotionDraft {

    private final double maxVelocity;
    private final double maxAcceleration;
    private State currentState = new State(0,0);
    private State goalState = new State(0,0);

    /**
     * This class is a better version of the TrapezoidalMotion class. It generates trapezoidal
     * motion profiles on the fly, rather than pre-generating them. This allows for more accurate
     * motion profiles, and allows for the robot to be able to change its motion profile in real-time.
     */
    public RealTimeTrapezoidalMotionDraft(double maxVelocity, double maxAcceleration){
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
    }

    public State calculate(double dt){
        State constranedState;
        var decelerationPosition = calcDecelerationPosition(dt);
        System.out.println("deceleration position: " + decelerationPosition);
        if(currentState.position >= decelerationPosition){
            //we are in the deceleration phase
            System.out.println("deceleration phase");
            constranedState = constrainDeceleration(currentState.position, goalState.position, dt);
        } else {
            //we are in the acceleration/coast phase
            System.out.println("acceleration phase");
            constranedState = constrainAcceleration(currentState.position, goalState.position, dt);
        }
        return constranedState;
    }

    public void setGoalState(double position, double velocity, double acceleration){
        goalState = new State(position, velocity);
    }

    public State constrainAcceleration(double initialPosition, double finalPosition, double dt){
        var velocity = finalPosition - initialPosition;
        var acceleration = velocity - currentState.velocity;

        //figure out which constraint is limiting the motion
        if(Math.abs(acceleration) > maxAcceleration){
            //acceleration is limiting the motion
            System.out.println("acceleration is limiting the motion");
            acceleration = Util.limit(acceleration, maxAcceleration);
            velocity = integrate(currentState.velocity, acceleration, dt);
        }
        if(Math.abs(velocity) > maxVelocity){
            velocity = Util.limit(velocity, maxVelocity);
            acceleration = derive(currentState.velocity, velocity, dt);
            //velocity is limiting the motion
            System.out.println("velocity is limiting the motion");
        }
        var position = integrate(initialPosition, velocity, dt);
        return new State(position, velocity);
    }

    public State constrainDeceleration(double initialPosition, double finalPosition, double dt){
        var velocity = finalPosition - initialPosition;
        var acceleration = -maxAcceleration;

        //figure out which constraint is limiting the motion
        if(Math.abs(acceleration) > maxAcceleration){
            //acceleration is limiting the motion
            System.out.println("acceleration is limiting the motion");
            acceleration = Util.limit(acceleration, maxAcceleration);
            velocity = integrate(currentState.velocity, acceleration, dt);
        }
        if(Math.abs(velocity) > maxVelocity){
            velocity = Util.limit(velocity, maxVelocity);
            acceleration = derive(currentState.velocity, velocity, dt);
            //velocity is limiting the motion
            System.out.println("velocity is limiting the motion");
        }
        var position = integrate(initialPosition, velocity, dt);
        return new State(position, velocity);
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

    public double derive(double initial, double integrated, double dt){
        return (integrated - initial) / dt;
    }

    public double integrateSlope(double initial, double slope, double dt){
        return integrate(initial*2, (dt/slope) - initial, dt) / 2;
    }

    public double deriveSlope(double integrated, double initial, double dt){
        return derive(initial/2, integrated, dt) * 2;
    }

   public double deriveInitial(double integrated, double slope, double dt){
        return derive(integrated, slope * 2, dt) / 2;
    }

    public double deriveDt(double integrated, double initial, double slope){
        return (2 * integrated) / (initial * slope);
    }


    public static void main(String[] args){
        var motion = new RealTimeTrapezoidalMotionDraft(1, 0.5);
//        System.out.println(motion.integrateSlope(1, 1, 2));
//        System.out.println(motion.deriveSlope(1, 0, 2));
//        System.out.println(motion.deriveInitial(1, 0, 2));
//        var pos = 0.0;
//        for(int i = 0; i < 100; i++){
//            var calc = motion.constrainAcceleration(pos, 5, 1);
//            System.out.println("Position: " + calc.position);
//            System.out.println("Velocity: " + calc.velocity);
//            System.out.println("Acceleration: " + calc.acceleration);
//            motion.setState(calc);
//            pos = calc.position;
//        }
        motion.setGoalState(5, 0, 0);
        motion.setState(new State(0, 0));
        motion.calcDecelerationPosition(1);
        //print the time to decelerate:
        System.out.println("time to decelerate: " + motion.calcTimeToDecelerate());
        for(int i = 0; i < 100; i++){
            var newState = motion.calculate(0.1);
            motion.setState(newState);
            System.out.println("Position: " + newState.position);
            System.out.println("Velocity: " + newState.velocity);
        }
    }

    public static final class State{
        public final double position;
        public final double velocity;

        public State(double position, double velocity){
            this.position = position;
            this.velocity = velocity;
        }
    }

    private enum Stage{
        TRACKING,
        DECELERATING,
    }
}
