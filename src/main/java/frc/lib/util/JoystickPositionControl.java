package frc.lib.util;

public class JoystickPositionControl {
    private final double sensitivity;
    private final double deadband;
    private double val = 0;

    /**
     * sipmle class to speed up linking a joystick to the
     * position of a robot component by integrating its value
     * @param sensitivity joystick sensitivity, in units/20ms
     * @param deadband joystick deadband
     */
    public JoystickPositionControl(double sensitivity, double deadband){
        this.sensitivity = sensitivity;
        this.deadband = deadband;
    }

    public void reset(double val){
        this.val = val;
    }

    /**
     * get the integrated joystick position
     * @param joystickPosition the joystick current position
     * @return the joystick position integrated over time
     */
    public double get(double joystickPosition){
        val += Util.handleDeadband(joystickPosition, deadband) * sensitivity;
        return val;
    }
}
