package frc.lib.util;

/**
 * A drivetrain command consisting of the left, right motor settings and whether the brake mode is enabled.
 */
public class DriveSignal {
    protected double left, right;
    protected boolean brakeMode, highGear;
    protected ControlMode mode;

    private static final boolean DEFAULT_BRAKE_MODE = true;

    public DriveSignal(double left, double right) {
        this(left, right, DEFAULT_BRAKE_MODE);
    }

    public DriveSignal(double left, double right, boolean brakeMode) {
        this(left, right, brakeMode, true, ControlMode.OPEN_LOOP);
    }

    public DriveSignal(double left, double right, boolean brakeMode, boolean highGear) {
        this(left, right, brakeMode, highGear, ControlMode.OPEN_LOOP);
    }

    public DriveSignal(double left, double right, boolean brakeMode, boolean highGear, ControlMode mode) {
        this.left = left;
        this.right = right;
        this.brakeMode = brakeMode;
        this.highGear = highGear;
        this.mode = mode;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(0, 0, false);
    public static DriveSignal BRAKE = new DriveSignal(0, 0, true);
    public static DriveSignal DEFAULT = new DriveSignal(0, 0, DEFAULT_BRAKE_MODE, true, ControlMode.OPEN_LOOP);

    public double getLeft() {
        return left;
    }

    public double getRight() {
        return right;
    }

    public boolean isBrakeMode() {
        return brakeMode;
    }

    public ControlMode getControlMode() {
        return mode;
    }

    public boolean isHighGear() {
        return highGear;
    }

    public static DriveSignal arcadeDrive(double throttle, double wheel, boolean highGear){
        return new DriveSignal(throttle + wheel, throttle - wheel, DEFAULT_BRAKE_MODE, highGear, ControlMode.OPEN_LOOP);
    }

    public static DriveSignal voltDrive(double left, double right, boolean highGear){
        return new DriveSignal(left, right, DEFAULT_BRAKE_MODE, highGear, ControlMode.VOLTAGE);
    }

    public static DriveSignal tankDrive(double left, double right, boolean highGear){
        return new DriveSignal(left, right, DEFAULT_BRAKE_MODE, highGear, ControlMode.OPEN_LOOP);
    }

    public static DriveSignal velocityDrive(double left, double right, boolean highGear){
        return new DriveSignal(left, right, DEFAULT_BRAKE_MODE, highGear, ControlMode.VELOCITY);
    }

    public static DriveSignal plus(DriveSignal a, DriveSignal b){
        if(a.mode != b.mode) throw new IllegalArgumentException("Cannot add two DriveSignals with different control modes!");
        return new DriveSignal(a.left + b.left, a.right + b.right, a.brakeMode, a.highGear, a.mode);
    }

    @Override
    public String toString() {
        return "L: " + left + ", R: " + right + (brakeMode ? ", BRAKE" : "");
    }

    public enum ControlMode{
        OPEN_LOOP, VOLTAGE, VELOCITY
    }
}