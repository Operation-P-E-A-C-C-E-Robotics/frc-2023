package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import static frc.robot.Constants.Constraints.*;

public class Constraints {
    SlewRateLimiter normalDriveLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT_NORMAL);
    SlewRateLimiter liftExtendedDriveLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT_LIFT_EXTENDED);
    private Kinematics kinematics;

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
}
