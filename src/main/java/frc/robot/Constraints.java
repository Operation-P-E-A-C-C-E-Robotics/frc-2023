package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.lib.util.DriveSignal;
import frc.robot.Kinematics.LiftPosition;
import frc.robot.Kinematics.LiftState;
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

    private static final double DRIVE_PITCH_THRESHOLD = 0, //todo
                               DRIVE_ROLL_THRESHOLD = 0, //todo
                               DRIVE_PITCH_RESET_THRESHOLD = 0, //todo
                               DRIVE_ROLL_RESET_THRESHOLD = 0, //todo
                               DRIVE_PITCH_CORRECTION_MULTIPLIER = 0.01, //todo
                               DRIVE_ROLL_CORRECTION_MULTIPLIER = 0.1; //todo

    private static final double DRIVE_SLEW_RATE_LIMIT_NORMAL = 0, //todo
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

    boolean resetRollDirection = true;
    boolean driveTrainDirection = true;
    double drivePitchAdjustedThreshold = DRIVE_PITCH_THRESHOLD;
    double driveRollAdjustedThreshold = DRIVE_ROLL_THRESHOLD;
    /**
     * Keep the drivetrain from falling over!
     * @param signal the drivesignal to constrain
     * @return the constrained drivesignal
     */
    public DriveSignal constrainDrivetrainTilt(DriveSignal signal){
        double pitch = odometry.getPitch();
        double roll = odometry.getRoll();

        //check if robot is too tilted
        if(Math.abs(pitch) < drivePitchAdjustedThreshold && Math.abs(roll) < driveRollAdjustedThreshold) {
            resetRollDirection = true;
            drivePitchAdjustedThreshold = DRIVE_PITCH_THRESHOLD;
            driveRollAdjustedThreshold = DRIVE_ROLL_THRESHOLD;
            return signal;
        }

        //lower the threshold to keep the reset from rapidly toggling on and off,
        //messing up the robot direction variable.
        drivePitchAdjustedThreshold = DRIVE_PITCH_RESET_THRESHOLD;
        driveRollAdjustedThreshold = DRIVE_ROLL_RESET_THRESHOLD;

        //correct for front/back tilt
        double pitchCorrectionFactor = pitch * DRIVE_PITCH_CORRECTION_MULTIPLIER;

        //figure out which way the drivetrain is moving, but only do it
        //once when the tilt first passes the threshold
        if(resetRollDirection) driveTrainDirection = (driveTrain.getWheelSpeeds().leftMetersPerSecond + driveTrain.getWheelSpeeds().rightMetersPerSecond) / 2 > 0;
        resetRollDirection = false;

        //correct for roll by moving opposite the robot direction
        double rollCorrectionFactor = roll * DRIVE_ROLL_CORRECTION_MULTIPLIER * (driveTrainDirection ? -1 : 1); //todo check direction

        return new DriveSignal(signal.getLeft() - pitchCorrectionFactor + rollCorrectionFactor, signal.getRight() - pitchCorrectionFactor + rollCorrectionFactor);
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
