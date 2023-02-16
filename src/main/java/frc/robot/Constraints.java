package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.util.Util;

import static frc.robot.Constants.Constraints.*;

public class Constraints {
    SlewRateLimiter normalDriveLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT_NORMAL);
    SlewRateLimiter liftExtendedDriveLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT_LIFT_EXTENDED);
    private Kinematics kinematics;

    /**
     * A class to hold functions that constrain the robot's behavior.
     * They take in a value and return the same value, modified to be within the limitations of the robot.
     * @param kinematics the robot's kinematics
     */
    public Constraints(Kinematics kinematics){
        this.kinematics = kinematics;
    }

    /**
     * constrain the joystick rate of change when driving to
     * help avoid tipping, especially when the arm is extended.
     * @param fwd the joystick's forward position
     * @return the corrected joystick forward position
     */
    public double constrainJoystickFwdJerk(double fwd){
        double normal = normalDriveLimiter.calculate(fwd);
        double extended = liftExtendedDriveLimiter.calculate(fwd);
        return kinematics.getSupersystemPosition().getX() > LIFT_EXTENDED_THRESHOLD ? extended : normal;
    }

    public static double ARM_EXTENSION_4FT = 0; //TODO how far the arm is extended to reach out 4 feet
    public static double MAX_X = 0; //TODO the maximum x value of the robot's end effector (4ft)
    public static double MAX_Y = 0; //TODO the maximum y value of the robot's end effector (4ft)
    public static double MAX_Z = 0; //TODO the maximum z value of the robot's end effector (max height)

    public static double PIVOT_COLLIDE_EXTENDED = 0; //TODO the angle at which the pivot makes the arm collide with the robot
    public static double PIVOT_COLLIDE_RETRACTED = 0; //TODO the angle at which the pivot makes the arm collide with the robot

    /**
     * prevent the arm from extending too far
     * @param state the supersystem state to constrain
     * @return the constrained supersystem state
     */
    public Kinematics.SupersystemState constrainArmExtension(Kinematics.SupersystemState state){
        //limit maximum extension of the arm
        if(state.getArmExtension() > ARM_EXTENSION_4FT){
            //the arm could be too far out, so use the kinematics to find the actual end effector position
            var prev = Kinematics.wristKinematics(state).getEndPosition();

            //limit the end effector position to the maximums
            var limited = new Translation3d(
                    Util.limit(prev.getX(), MAX_X),
                    Util.limit(prev.getY(), MAX_Y),
                    Util.limit(prev.getZ(), MAX_Z)
            );
            state = Kinematics.inverseKinematicsFromEndEffector(limited, state.getWristAngle());
        }
        return state;
    }

    /**
     * constrain the current supersystem state to prevent the robot from colliding with itself
     * @param state the supersystem state to constrain
     * @return the constrained supersystem state
     */
    public static Kinematics.SupersystemState constrainPivotCollision(Kinematics.SupersystemState state){
        //keep the pivot from colliding with the robot
        if(Math.abs(state.getPivotAngle()) > PIVOT_COLLIDE_EXTENDED){
            //if the pivot is within the collision range, move the arm in
            var newArm = 0; //TODO the arm all the way in.
            double newPivot = state.getPivotAngle();
            if(Math.abs(state.getPivotAngle()) > PIVOT_COLLIDE_RETRACTED){
                //if the arm will still collide with the robot, keep the pivot at the collision angle
                newPivot = Math.signum(state.getPivotAngle()) * PIVOT_COLLIDE_RETRACTED;
            }
            state = new Kinematics.SupersystemState(state.getTurretAngle(), newPivot, newArm, state.getWristAngle());
        }
        return state;
    }
}
