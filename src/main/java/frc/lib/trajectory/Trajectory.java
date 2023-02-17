package frc.lib.trajectory;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import static edu.wpi.first.math.trajectory.TrapezoidProfile.*;

public class Trajectory {
    private Motion[] waypoints;
    private final State initialState;

    public Trajectory(State... waypoints){
        this.initialState = waypoints[0];
        this.waypoints = new Motion[waypoints.length - 1];
        for(int i = 1; i < waypoints.length; i++){
            this.waypoints[i - 1] = Motion.fromState(waypoints[i - 1], waypoints[i]);
        }
    }

    public State calculate(double time){
        var waypoint = waypoints[0];
        var timeInWaypoint = 0.0;
        var position = initialState.position;
        for (Motion motion : waypoints) {
            if (timeInWaypoint + motion.time > time) {
                waypoint = motion;
                break;
            }
            position += motion.deltaPosition;
            timeInWaypoint += motion.time;
        }
        var interpolated = waypoint.interpolateTime(time - timeInWaypoint);
        return new State(position + interpolated.deltaPosition, interpolated.initialVelocity);
    }

    public double getTotalTime(){
        var time = 0.0;
        for (Motion motion : waypoints) {
            time += motion.time;
        }
        return time;
    }

    public void append(Trajectory other){
        var newWaypoints = new Motion[waypoints.length + other.waypoints.length];
        System.arraycopy(waypoints, 0, newWaypoints, 0, waypoints.length);
        System.arraycopy(other.waypoints, 0, newWaypoints, waypoints.length, other.waypoints.length);
        waypoints = newWaypoints;
    }

    public static Trajectory trapezoidTrajectory(State current, State target, double maxVelocity, double maxAcceleration){
        //calculate the distance to accelerate and coast
        var deltaPosition = Math.abs(target.position - current.position);
        var signum = Math.signum(target.position - current.position);
        Trajectory trajectory;

        // if the current velocity is in the opposite direction of the target, decelerate to 0 and then accelerate
        // if we don't do this, the math gets pissy.
        if(signum != Math.signum(current.velocity) && signum != 0 && Math.signum(current.velocity) != 0){
            //calculate the distance to decelerate from the current velocity to 0:
            var decelerationDistance = Math.pow(current.velocity, 2) / (2 * maxAcceleration);

            //calculate the two trajectories
            var intermediate = new State(current.position + decelerationDistance * Math.signum(current.velocity), 0);
            var t1 = Trajectory.trapezoidTrajectory(current, intermediate, maxVelocity, maxAcceleration);
            var t2 = Trajectory.trapezoidTrajectory(intermediate, target, maxVelocity, maxAcceleration);

            //append the two trajectories
            trajectory = t1;
            trajectory.append(t2);
            return trajectory;
        }

        var accelerationDistance = Math.pow(maxVelocity, 2) / (2 * maxAcceleration);
        var coastDistance = deltaPosition - 2 * accelerationDistance;

        // if the distance it will take to accelerate is higher than the distance to the target, accelerate
        // to a lower velocity and don't coast (triangular profile)
        if(accelerationDistance > deltaPosition/2){
            maxVelocity = Math.sqrt(2 * maxAcceleration * deltaPosition);
            accelerationDistance = Math.pow(maxVelocity, 2) / (2 * maxAcceleration) / 2;
            coastDistance = 0;
        }

        accelerationDistance *= signum;
        coastDistance *= signum;
        maxVelocity *= signum;

        return new Trajectory(
                current,
                new State(current.position + accelerationDistance, maxVelocity),
                new State(current.position + accelerationDistance + coastDistance, maxVelocity),
                target
        );
    }

    public static void main(String[] args){
        var test = new Trajectory(
                new State(5,-5),
                new State(-1,-1),
                new State(-2,-1),
                new State(-3,0),
                new State(-2,1)
        );
        for(double i = 0; i <= test.getTotalTime(); i += 0.01){
            System.out.println(test.calculate(i).position);
        }
    }
}
