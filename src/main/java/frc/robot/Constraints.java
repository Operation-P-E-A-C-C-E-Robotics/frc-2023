package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.Util;
import frc.robot.Constants.Arm;
import frc.robot.subsystems.Supersystem;

import static frc.robot.Constants.Constraints.*;

public class Constraints {
    private final SlewRateLimiter normalDriveLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT_NORMAL);
    private final SlewRateLimiter liftExtendedDriveLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT_LIFT_EXTENDED);
    private final Kinematics kinematics;
    double prevFwd = 0;

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
        if(Math.abs(prevFwd) > Math.abs(fwd)) return fwd;
        prevFwd = fwd;
        double normal = normalDriveLimiter.calculate(fwd);
        double extended = liftExtendedDriveLimiter.calculate(fwd);
        return Util.interpolate(normal, extended, (kinematics.getSupersystemState().getArmExtension() - Arm.MIN_EXTENSION) / (Arm.MAX_EXTENSION - Arm.MIN_EXTENSION));
    }

    public static double ARM_EXTENSION_4FT = Units.feetToMeters(4); //TODO how far the arm is extended to reach out 4 feet
    public static double MAX_X = Units.feetToMeters(4); //TODO add frame perimeter to this
    public static double MAX_Y = Units.feetToMeters(4);
    public static double MAX_Z = Units.feetToMeters(6); //TODO the maximum z value of the robot's end effector (max height)


    public static double PIVOT_COLLIDE_RETRACTED = Constants.Pivot.MAX_ANGLE_RAD;

    /**
     * prevent the arm from extending too far
     * @param state the supersystem state to constrain
     * @return the constrained supersystem state
     */
    public static Kinematics.SupersystemState constrainArmExtension(Kinematics.SupersystemState state){
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
        double newPivot = state.getPivotAngle();
        if(Math.abs(state.getPivotAngle()) > PIVOT_COLLIDE_RETRACTED){
            //if the arm will still collide with the robot, keep the pivot at the collision angle
            newPivot = Math.signum(state.getPivotAngle()) * PIVOT_COLLIDE_RETRACTED;
        }
        state = new Kinematics.SupersystemState(state.getTurretAngle(), newPivot, state.getArmExtension(), state.getWristAngle());
        return state;
    }
}
