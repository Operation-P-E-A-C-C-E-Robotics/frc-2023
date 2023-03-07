package frc.lib.util;

import edu.wpi.first.math.filter.SlewRateLimiter;

public class PeaccyDriveHelper {
    private static final double THROTTLE_DEADBAND = 0.035, WHEEL_DEADBAND = 0.02;
    private static final double HIGH_WEEL_NON_LINEARITY = 0.6,
            HIGH_SENSITIVITY = 0.7,
            HIGH_NEG_INERTIA_SCALAR = 3.0; //how sensitive the robot is to the change in wheel
    private static final double LOW_WHEEL_NON_LINEARITY = 0.5,
            LOW_SENSITIVITY = 0.65,
            LOW_NEG_INERTIA_THRESHOLD = 0.65,  //when we switch from the close scalar to the far scalar
            LOW_NEG_INERTIA_TURN_SCALAR = 3.5,  //for increase in wheel
            LOW_NEG_INERTIA_CLOSE_SCALAR = 4.0,  //for decrease in wheel close to zero
            LOW_NEG_INERTIA_FAR_SCALAR = 5.0; //for decrease in wheel far from zero
    private static final double QUICK_STOP_DEADBAND = 0.5,
            QUICK_STOP_WEIGHT = 0.3,
            QUICK_STOP_SCALAR = 5.0,
            QUICK_TURN_SCALAR = 0.65;

    private double prevWheel = 0.0, negInertiaAccumulator = 0.0, quickStopAccumulator = 0.0;

    public static final double FINE_TURN_SENSITIVITY = 0.1, FINE_THROTTLE_SENSITIVITY = 0.1;

    private static final SlewRateLimiter fastAccelLimiterLeft = new SlewRateLimiter(0.5); //TODO
    private static final SlewRateLimiter fastAccelLimiterRight = new SlewRateLimiter(0.5); //TODO
    private static final double FAST_ACCEL_UPSHIFT_THRESHOLD = 0.5,
            FAST_ACCEL_DOWNSHIFT_THRESHOLD = 0.3;
    private static final double FAST_ACCEL_SCALAR = 0.5; //TODO

    /**
     * functionally identical to cheesy drive.
     * @param throttle forward percentage
     * @param wheel arc radius
     * @param arcadeWheel additional turn
     * @param quickturn enable quickturn
     * @param highGear enable high gear
     * @return drive signal
     */
    public DriveSignal curveDrive(double throttle, double wheel, double arcadeWheel, boolean quickturn, boolean highGear){
        double left, right;
        double angularPower;
        double sensitivity = highGear ? HIGH_SENSITIVITY : LOW_SENSITIVITY;
        double overPower = highGear ? 0.0 : 0.3;

        throttle = Util.handleDeadband(throttle, THROTTLE_DEADBAND);
        wheel = Util.handleDeadband(wheel, WHEEL_DEADBAND);
        wheel = applySinCurve(wheel, highGear);

        var deltaWheel = wheel - prevWheel;
        prevWheel = wheel;

        wheel = applyNegInertia(wheel, deltaWheel, highGear);

        angularPower = quickturn ? getQuickturnAngularPower(throttle, wheel) : getAngularPower(throttle, wheel, sensitivity);
        angularPower += arcadeWheel;

        left = throttle + angularPower;
        right = throttle - angularPower;
        return applyOverPower(new DriveSignal(left, right, true, highGear), overPower);
    }


    /**
     * use velocity control + low gear for fine maneuvering
     * @param throttle forward percentage
     * @param wheel turn speed
     * @return drive signal
     */
    public DriveSignal fineControl(double throttle, double wheel){
        throttle = applySinCurve(throttle, false);
        wheel = applySinCurve(wheel, false);
        throttle *= FINE_THROTTLE_SENSITIVITY;
        wheel *= FINE_TURN_SENSITIVITY;
        return new DriveSignal(throttle + wheel, throttle - wheel, false, false, DriveSignal.ControlMode.VELOCITY);
    }

    /**
     * use velocity control with a slew rate limiter and auto shifting to
     * accelerate quickly and smoothly
     * @param throttle forward percentage
     * @param wheel arc radius
     * @param velocity current velocity
     * @param currentGear current gear
     * @return drive signal
     */
    public DriveSignal fastAcceleration(double throttle, double wheel, double velocity, boolean currentGear){
        DriveSignal setpoints = curveDrive(throttle, wheel, 0, false, currentGear);
        var left = fastAccelLimiterLeft.calculate(setpoints.getLeft() * FAST_ACCEL_SCALAR);
        var right = fastAccelLimiterRight.calculate(setpoints.getRight() * FAST_ACCEL_SCALAR);

        if(velocity > FAST_ACCEL_UPSHIFT_THRESHOLD) return new DriveSignal(left, right, true, true, DriveSignal.ControlMode.VELOCITY);
        else if(velocity < FAST_ACCEL_DOWNSHIFT_THRESHOLD) return new DriveSignal(left, right, true, false, DriveSignal.ControlMode.VELOCITY);
        else return new DriveSignal(left, right, true, currentGear, DriveSignal.ControlMode.VELOCITY);
    }

    /**
     * get the angular power for a quickturn
     * @param throttle forward percentage
     * @param wheel turn speed
     * @return angular power
     */
    private double getQuickturnAngularPower(double throttle, double wheel){
        if(Math.abs(throttle) < QUICK_STOP_DEADBAND){
            quickStopAccumulator = (1 - QUICK_STOP_WEIGHT) * quickStopAccumulator + QUICK_STOP_WEIGHT * Util.limit(wheel, 1.0) * QUICK_STOP_SCALAR;
        }
        return wheel * QUICK_TURN_SCALAR;
    }

    /**
     * get angular power for a curvature drive
     * @param throttle forward percentage
     * @param wheel turn speed
     * @param sensitivity sensitivity
     * @return angular power
     */
    private double getAngularPower(double throttle, double wheel, double sensitivity){
        var angularPower = Math.abs(throttle) * wheel * sensitivity - quickStopAccumulator;
        if(Util.inRange(quickStopAccumulator, 1)) quickStopAccumulator = 0;
        else quickStopAccumulator -= Math.signum(quickStopAccumulator);
        return angularPower;
    }

    /**
     * subtract unobtainable signals from the opposite side of the drivetrain,
     * to keep turning more consistent.
     * @param signal drive signal
     * @param overPower percentage of signal to subtract
     * @return drive signal
     */
    private DriveSignal applyOverPower(DriveSignal signal, double overPower){
        var left = signal.getLeft();
        var right = signal.getRight();
        if(left > 1.0) {
            right -= overPower * (left - 1.0);
            left = 1.0;
        } else if(right < -1.0) {
            left += overPower * (-1.0 - right);
            right = -1.0;
        } else if(right > 1.0) {
            left -= overPower * (right - 1.0);
            right = 1.0;
        } else if(left < -1.0) {
            right += overPower * (-1.0 - left);
            left = -1.0;
        }
        return new DriveSignal(left, right, signal.isBrakeMode(), signal.isHighGear());
    }

    /**
     * apply negative inertia to the wheel (increase it when the wheel is changing,
     * to make the robot react faster)
     * @param wheel current wheel
     * @param deltaWheel change in wheel
     * @param gear current gear
     * @return new wheel
     */
    private double applyNegInertia(double wheel, double deltaWheel, boolean gear){
        negInertiaAccumulator += deltaWheel * getNegInertiaScalar(wheel, deltaWheel, gear);
        wheel += negInertiaAccumulator;

        if(Util.inRange(negInertiaAccumulator, 1)) negInertiaAccumulator = 0;
        else negInertiaAccumulator -= Math.signum(negInertiaAccumulator);

        return wheel;
    }

    private double getNegInertiaScalar(double wheel, double deltaWheel, boolean gear){
        if(gear) return HIGH_NEG_INERTIA_SCALAR;
        if (wheel * deltaWheel > 0) return LOW_NEG_INERTIA_TURN_SCALAR; // If we are moving away from 0.0, aka, trying to get more wheel.
        if (Math.abs(wheel) > LOW_NEG_INERTIA_THRESHOLD) return LOW_NEG_INERTIA_FAR_SCALAR; // Otherwise, we are attempting to go back to 0.0.
        return LOW_NEG_INERTIA_CLOSE_SCALAR;
    }

    private double applySinCurve(double wheel, boolean gear){
        var nonLinearity = gear ? HIGH_WEEL_NON_LINEARITY : LOW_WHEEL_NON_LINEARITY;
        var denominator = Math.sin(Math.PI / 2.0 * nonLinearity);
        wheel = Math.sin(Math.PI / 2.0 * nonLinearity * wheel) / denominator;
        wheel = Math.sin(Math.PI / 2.0 * nonLinearity * wheel) / denominator;
        if(!gear) wheel = Math.sin(Math.PI / 2.0 * nonLinearity * wheel) / denominator; //extra one for low gear i guess
        return wheel;
    }
}
