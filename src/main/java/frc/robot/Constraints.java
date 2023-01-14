package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.Kinematics.LiftPosition;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Lift;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Wrist;

public class Constraints {
    private Lift lift;
    private Pivot pivot;
    private Turret turret;
    private Wrist wrist;
    private DriveTrain driveTrain;
    private Odometry odometry;

    private static final double DRIVE_SLEW_RATE_LIMIT_NORMAL = 10, //todo
                               DRIVE_SLEW_RATE_LIMIT_LIFT_EXTENDED = 0, //todo
                               LIFT_EXTENDED_THRESHOLD = 0.4; //todo

    public Constraints(DriveTrain driveTrain, Lift lift, Pivot pivot, Turret turret, Wrist wrist, Odometry odometry){
        this.driveTrain = driveTrain;
        this.lift = lift;
        this.pivot = pivot;
        this.turret = turret;
        this.wrist = wrist;
        this.odometry = odometry;
    }

    SlewRateLimiter normalDriveLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT_NORMAL);
    SlewRateLimiter liftExtendedDrieLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT_LIFT_EXTENDED);

    /**
     * constrain the joystick rate of change when driving to
     * help avoid tipping, especially when the arm is extended.
     * @param fwd the joystick's forward postion
     * @return the corrected joystick forward position
     */
    public double constrainJoystickFwdJerk(double fwd){
        double normal = normalDriveLimiter.calculate(fwd);
        double extended = liftExtendedDrieLimiter.calculate(fwd);
        return GET_LIFT_X_FROM_KINEMATICS_PLACEHOLDER() > LIFT_EXTENDED_THRESHOLD ? extended : normal;
    }
    private double GET_LIFT_X_FROM_KINEMATICS_PLACEHOLDER(){
        return 0; //TODO
    }
    public void constrainHandlerState(LiftPosition currentPose, LiftPosition targetPose){

    }
}
