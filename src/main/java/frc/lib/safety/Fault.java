package frc.lib.safety;

public class Fault {
    private final String name;
    private final FaultType type;
    private final Severity severity;
    private final String message;
    public Fault(String name, FaultType type, Severity severity, String message){
        this.name = name;
        this.type = type;
        this.severity = severity;
        this.message = message;
    }
    public String getName(){
        return name;
    }
    public FaultType getType(){
        return type;
    }
    public Severity getSeverity(){
        return severity;
    }
    public String getMessage(){
        return message;
    }
    public String toString(){
        return type.toString() + " FAULT [" + severity.toString() + "] - " +
                name + ": '" + message + "'";
    }
    public enum FaultType {
        HARDWARE, SOFTWARE
    }
    public enum Severity{
        NONE, WARNING, ERROR, FATAL
    }

    public static void main(String[] args){
        Fault f = new Fault("test", FaultType.HARDWARE, Severity.WARNING, "test message");
        System.out.println(f);
    }
}
