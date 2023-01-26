package frc.lib.util;

/**
 * Helper class to implement "Cheesy Drive". "Cheesy Drive" simply means that the "turning" stick controls the curvature
 * of the robot's path rather than its rate of heading change. This helps make the robot more controllable at high
 * speeds. Also handles the robot's quick turn functionality - "quick turn" overrides constant-curvature turning for
 * turn-in-place maneuvers.
 */
public class CheesyDriveHelper {

    private static final double THROTTLE_DEADBAND = 0.02;
    private static final double WHEEL_DEADBAND = 0.02;

    // These factor determine how fast the wheel traverses the "non-linear" sine curve.
    private static final double HIGH_WHEEL_NON_LINEARITY = 0.7;
    private static final double LOW_WHEEL_NON_LINEARITY = 0.5;

    private static final double kHighNegInertiaScalar = 5;

    private static final double LOW_NEG_INERTIA_THRESHOLD = 0.65;
    private static final double LOW_NEG_INERTIA_TURN_SCALAR = 3.5;
    private static final double LOW_NEG_INERTIA_CLOSE_SCALAR = 4.0;
    private static final double LOW_NEG_INERTIA_FAR_SCALAR = 5.0;

    private static final double kHighSensitivity = 0.8;
    private static final double kLowSensitiity = 0.65;

    private static final double QUICK_STOP_DEADBAND = 0.5;
    private static final double QUICK_STOP_WEIGHT = 0.1;
    private static final double QUICK_STOP_SCALAR = 5.0;

    private double mOldWheel = 0.0;
    private double mQuickStopAccumlator = 0.0;
    private double mNegInertiaAccumlator = 0.0;

    public DriveSignal cheesyDrive(double throttle, double wheel, boolean isQuickTurn,
                                   boolean isHighGear) {

        wheel = handleDeadband(wheel, WHEEL_DEADBAND);
        throttle = handleDeadband(throttle, THROTTLE_DEADBAND);

        double negInertia = wheel - mOldWheel;
        mOldWheel = wheel;

        double wheelNonLinearity;
        if (isHighGear) {
            wheelNonLinearity = HIGH_WHEEL_NON_LINEARITY;
            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        } else {
            wheelNonLinearity = LOW_WHEEL_NON_LINEARITY;
            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
            wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel) / denominator;
        }

        double leftPwm, rightPwm, overPower;
        double sensitivity;

        double angularPower;
        double linearPower;

        // Negative inertia!
        double negInertiaScalar;
        if (isHighGear) {
            negInertiaScalar = kHighNegInertiaScalar;
            sensitivity = kHighSensitivity;
        } else {
            if (wheel * negInertia > 0) {
                // If we are moving away from 0.0, aka, trying to get more wheel.
                negInertiaScalar = LOW_NEG_INERTIA_TURN_SCALAR;
            } else {
                // Otherwise, we are attempting to go back to 0.0.
                if (Math.abs(wheel) > LOW_NEG_INERTIA_THRESHOLD) {
                    negInertiaScalar = LOW_NEG_INERTIA_FAR_SCALAR;
                } else {
                    negInertiaScalar = LOW_NEG_INERTIA_CLOSE_SCALAR;
                }
            }
            sensitivity = kLowSensitiity;
        }
        double negInertiaPower = negInertia * negInertiaScalar;
        mNegInertiaAccumlator += negInertiaPower;

        wheel = wheel + mNegInertiaAccumlator;
        if (mNegInertiaAccumlator > 1) {
            mNegInertiaAccumlator -= 1;
        } else if (mNegInertiaAccumlator < -1) {
            mNegInertiaAccumlator += 1;
        } else {
            mNegInertiaAccumlator = 0;
        }
        linearPower = throttle;

        // Quickturn!
        if (isQuickTurn) {
            if (Math.abs(linearPower) < QUICK_STOP_DEADBAND) {
                double alpha = QUICK_STOP_WEIGHT;
                mQuickStopAccumlator = (1 - alpha) * mQuickStopAccumlator
                        + alpha * Util.limit(wheel, 1.0) * QUICK_STOP_SCALAR;
            }
            overPower = 0.5;
            angularPower = wheel;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * wheel * sensitivity - mQuickStopAccumlator;
            if (mQuickStopAccumlator > 1) {
                mQuickStopAccumlator -= 1;
            } else if (mQuickStopAccumlator < -1) {
                mQuickStopAccumlator += 1;
            } else {
                mQuickStopAccumlator = 0.0;
            }
        }

        rightPwm = leftPwm = linearPower;
        leftPwm += angularPower;
        rightPwm -= angularPower;

        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }
        return new DriveSignal(leftPwm, rightPwm);
    }

    public double handleDeadband(double val, double deadband) {
        return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
    }
}