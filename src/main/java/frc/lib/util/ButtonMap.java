package frc.lib.util;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    public static class MultiButton implements OIEntry {
        public final Command command;
        public final int[] buttonNumbers;
        public final TriggerType triggerType;
        public MultiButton(Command command, TriggerType triggerType, int... buttonNumbers){
            this.command = command;
            this.buttonNumbers = buttonNumbers;
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
            Trigger[] triggers = new Trigger[buttonNumbers.length];
            for(int i = 0; i < buttonNumbers.length; i++){
                triggers[i] = new JoystickButton(joystick, buttonNumbers[i]);
            }
            return new Trigger(() -> {
                    for(Trigger trigger : triggers){
                        if(!trigger.getAsBoolean()){
                            return false;
                        }
                    }
                    return true;
            });
        }

        public static MultiButton onHold(Command command, int... buttonNumbers){
            return new MultiButton(command, TriggerType.WHILE_HOLD, buttonNumbers);
        }

        public static MultiButton onPress(Command command, int... buttonNumbers){
            return new MultiButton(command, TriggerType.ON_PRESS, buttonNumbers);
        }

        public static MultiButton onRelease(Command command, int... buttonNumbers){
            return new MultiButton(command, TriggerType.ON_RELEASE, buttonNumbers);
        }

        public static MultiButton toggle(Command command, int... buttonNumbers){
            return new MultiButton(command, TriggerType.TOGGLE, buttonNumbers);
        }
    }

    public static class OIMode implements OIEntry{
        private final int selectButtonNumber;
        private final OIEntry[] entries;
        private final Joystick joystick;
        private boolean selected;

        public OIMode(int selectButtonNumber, boolean defaultSelected, Joystick joystick, OIEntry... entries){
            this.selectButtonNumber = selectButtonNumber;
            this.entries = entries;
            this.joystick = joystick;
            selected = defaultSelected;
        }

        @Override
        public void bindTo(Trigger trigger) {
            trigger.onTrue(new InstantCommand(() -> selected = !selected));
            for(OIEntry entry : entries){
                entry.bindTo(entry.getTrigger(joystick).and(new Trigger(() -> selected)));
            }
        }

        @Override
        public Trigger getTrigger(Joystick joystick) {
            return new JoystickButton(joystick, selectButtonNumber);
        }
    }

    public static class SimplePOV implements OIEntry{
        public final Command command;
        public final int povNumber;
        public final TriggerType triggerType;

        public SimplePOV(Command command, int povNumber, TriggerType triggerType){
            this.command = command;
            this.povNumber = povNumber;
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
            return new Trigger(() -> joystick.getPOV() == povNumber);
        }

        public static SimplePOV onHold(Command command, int povNumber){
            return new SimplePOV(command, povNumber, TriggerType.WHILE_HOLD);
        }

        public static SimplePOV onPress(Command command, int povNumber){
            return new SimplePOV(command, povNumber, TriggerType.ON_PRESS);
        }

        public static SimplePOV onRelease(Command command, int povNumber){
            return new SimplePOV(command, povNumber, TriggerType.ON_RELEASE);
        }

        public static SimplePOV toggle(Command command, int povNumber){
            return new SimplePOV(command, povNumber, TriggerType.TOGGLE);
        }
    }

    public static class FancyPOVAndButton implements OIEntry{
        public final Command command;
        public final int buttonNumber;
        public final TriggerType triggerType;

        public FancyPOVAndButton(Command command, int buttonNumber, TriggerType triggerType){
            this.command = command;
            this.buttonNumber = buttonNumber;
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
            Trigger povTrigger = new Trigger(() -> joystick.getPOV() != -1);
            return new JoystickButton(joystick, buttonNumber).and(povTrigger);
        }
    }

    public static enum TriggerType {
        ON_PRESS, ON_RELEASE, WHILE_HOLD, TOGGLE
    }
}
