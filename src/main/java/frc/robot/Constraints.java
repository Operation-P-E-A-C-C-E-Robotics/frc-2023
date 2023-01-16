package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.subsystems.*;

public class Constraints {
    @SuppressWarnings("FieldCanBeLocal")
    private final Arm arm;
    @SuppressWarnings("FieldCanBeLocal")
    private final Pivot pivot;
    @SuppressWarnings("FieldCanBeLocal")
    private final Turret turret;
    @SuppressWarnings("FieldCanBeLocal")
    private final Wrist wrist;
    @SuppressWarnings("FieldCanBeLocal")
    private final DriveTrain driveTrain;
    @SuppressWarnings("FieldCanBeLocal")
    private final RobotState robotState;

    private static final double DRIVE_SLEW_RATE_LIMIT_NORMAL = 10, //todo
                               DRIVE_SLEW_RATE_LIMIT_LIFT_EXTENDED = 0, //todo
                               LIFT_EXTENDED_THRESHOLD = 0.4; //todo

    public Constraints(DriveTrain driveTrain, Arm arm, Pivot pivot, Turret turret, Wrist wrist, RobotState robotState){
        this.driveTrain = driveTrain;
        this.arm = arm;
        this.pivot = pivot;
        this.turret = turret;
        this.wrist = wrist;
        this.robotState = robotState;
    }

    SlewRateLimiter normalDriveLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT_NORMAL);
    SlewRateLimiter liftExtendedDriveLimiter = new SlewRateLimiter(DRIVE_SLEW_RATE_LIMIT_LIFT_EXTENDED);

    /**
     * constrain the joystick rate of change when driving to
     * help avoid tipping, especially when the arm is extended.
     * @param fwd the joystick's forward position
     * @return the corrected joystick forward position
     */
    public double constrainJoystickFwdJerk(double fwd){
        double normal = normalDriveLimiter.calculate(fwd);
        double extended = liftExtendedDriveLimiter.calculate(fwd);
        return GET_LIFT_X_FROM_KINEMATICS_PLACEHOLDER() > LIFT_EXTENDED_THRESHOLD ? extended : normal;
    }
    private double GET_LIFT_X_FROM_KINEMATICS_PLACEHOLDER(){
        return 0; //TODO
    }
}
