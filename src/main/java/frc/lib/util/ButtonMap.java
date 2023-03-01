package frc.lib.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * A class to help with mapping joysticks to commands.
 */
public class ButtonMap {
    private final Joystick joystick;

    public ButtonMap(Joystick joystick) {
        this.joystick = joystick;
    }

    public void map(OIEntry... entries){
        for(OIEntry entry : entries){
            JoystickButton button = new JoystickButton(joystick, entry.buttonNumber);
            switch (entry.trigger) {
                case ON_PRESS -> button.onTrue(entry.command);
                case ON_RELEASE -> button.onFalse(entry.command);
                case WHILE_HOLD -> button.whileTrue(entry.command);
                case TOGGLE -> button.toggleOnTrue(entry.command);
            }
        }
    }

    public static class OIEntry{
        public final Command command;
        public final int buttonNumber;
        public final Trigger trigger;
        public OIEntry(Command command, int buttonNumber, Trigger trigger){
            this.command = command;
            this.buttonNumber = buttonNumber;
            this.trigger = trigger;
        }

        public static OIEntry onHold(Command command, int buttonNumber){
            return new OIEntry(command, buttonNumber, Trigger.WHILE_HOLD);
        }

        public static OIEntry onPress(Command command, int buttonNumber){
            return new OIEntry(command, buttonNumber, Trigger.ON_PRESS);
        }

        public static OIEntry onRelease(Command command, int buttonNumber){
            return new OIEntry(command, buttonNumber, Trigger.ON_RELEASE);
        }

        public static OIEntry toggle(Command command, int buttonNumber){
            return new OIEntry(command, buttonNumber, Trigger.TOGGLE);
        }

        enum Trigger{
            ON_PRESS, ON_RELEASE, WHILE_HOLD, TOGGLE
        }
    }
}
