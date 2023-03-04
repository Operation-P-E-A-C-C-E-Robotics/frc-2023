package frc.lib.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
            var button = entry.getTrigger(joystick);
            entry.bindTo(button);
        }
    }

    public static interface OIEntry{
        public void bindTo(Trigger trigger);
        public Trigger getTrigger(Joystick joystick);
    }

    public static class SimpleButton implements OIEntry{
        public final Command command;
        public final int buttonNumber;
        public final TriggerType triggerType;
        public SimpleButton(Command command, int buttonNumber, TriggerType triggerType){
            this.command = command;
            this.buttonNumber = buttonNumber;
            this.triggerType = triggerType;
        }

        public void bindTo(Trigger trigger){
            switch (triggerType) {
                case ON_PRESS -> trigger.onTrue(command);
                case ON_RELEASE -> trigger.onFalse(command);
                case WHILE_HOLD -> trigger.whileTrue(command);
                case TOGGLE -> trigger.toggleOnTrue(command);
            }
        }

        public Trigger getTrigger(Joystick joystick){
            return new JoystickButton(joystick, buttonNumber);
        }

        public static SimpleButton onHold(Command command, int buttonNumber){
            return new SimpleButton(command, buttonNumber, TriggerType.WHILE_HOLD);
        }

        public static SimpleButton onPress(Command command, int buttonNumber){
            return new SimpleButton(command, buttonNumber, TriggerType.ON_PRESS);
        }

        public static SimpleButton onRelease(Command command, int buttonNumber){
            return new SimpleButton(command, buttonNumber, TriggerType.ON_RELEASE);
        }

        public static SimpleButton toggle(Command command, int buttonNumber){
            return new SimpleButton(command, buttonNumber, TriggerType.TOGGLE);
        }


    }

    public static class FancyPOVAndButton implements OIEntry{
        public final Command command;
        public final int buttonNumber;
        public final int povAngle;
        public final TriggerType triggerType;

        public FancyPOVAndButton(Command command, int buttonNumber, int povAngle, TriggerType triggerType){
            this.command = command;
            this.buttonNumber = buttonNumber;
            this.povAngle = povAngle;
            this.triggerType = triggerType;
        }

        @Override
        public void bindTo(Trigger trigger) {
            switch (triggerType) {
                case ON_PRESS -> trigger.onTrue(command);
                case ON_RELEASE -> trigger.onFalse(command);
                case WHILE_HOLD -> trigger.whileTrue(command);
                case TOGGLE -> trigger.toggleOnTrue(command);
            }
        }

        @Override
        public Trigger getTrigger(Joystick joystick) {
            Trigger povTrigger = new Trigger(() -> joystick.getPOV() == povAngle);
            return new JoystickButton(joystick, buttonNumber).and(povTrigger);
        }
    }

    public static enum TriggerType {
        ON_PRESS, ON_RELEASE, WHILE_HOLD, TOGGLE
    }
}
