package frc.robot.commands.supersystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.subsystems.Supersystem;

public class Setpoints {
    public static Kinematics.SupersystemState placeLow = new Kinematics.SupersystemState(0, 0, 0, 0);
    public static Kinematics.SupersystemState placeMidCube = new Kinematics.SupersystemState(0, 0, 0, 0);
    public static Kinematics.SupersystemState placeHighCube = new Kinematics.SupersystemState(0, 0.5, 1.5, 10);
    public static Kinematics.SupersystemState placeMidCone = new Kinematics.SupersystemState(0, 0, 0, 0);
    public static Kinematics.SupersystemState placeHighCone = new Kinematics.SupersystemState(0, 0, 0, 0);
    public static Kinematics.SupersystemState intakeFloor = new Kinematics.SupersystemState(0, 0, 0, 0);
    public static Kinematics.SupersystemState intakeDoubleSubstation = new Kinematics.SupersystemState(0, 0, 0, 0);

    public static Command goToSetpoint(Kinematics.SupersystemState setpoint, Supersystem supersystem, Constants.SupersystemTolerance tolerance){
        return new RunCommand(
                () -> supersystem.setSupersystemState(setpoint), supersystem
        );
    }
}
