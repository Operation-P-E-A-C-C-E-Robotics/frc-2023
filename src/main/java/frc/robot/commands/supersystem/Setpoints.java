package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.subsystems.Supersystem;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

import static frc.robot.Kinematics.*;

public class Setpoints {
    public static SupersystemState placeLow = new SupersystemState(0, 0, 0, 0);
    public static SupersystemState placeMidCube = new SupersystemState(0, 0, 0, 0);
    public static SupersystemState placeHighCube = new SupersystemState(0, 0.5, 1.5, 10);
    public static SupersystemState placeMidCone = new SupersystemState(0, 0, 0, 0);
    public static SupersystemState placeHighCone = new SupersystemState(0, 0, 0, 0);
    public static SupersystemState intakeFloor = new SupersystemState(0, 0, 0, 0);
    public static SupersystemState intakeDoubleSubstation = new SupersystemState(0, 0, 0, 0);
    private final Supersystem supersystem;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier z;
    private final DoubleSupplier turret;

    public Setpoints(Supersystem supersystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier z, DoubleSupplier turret){
        this.supersystem = supersystem;
        this.x = x;
        this.y = y;
        this.z = z;
        this.turret = turret;
    }

    public Command goToSetpoint(SupersystemState setpoint, Constants.SupersystemTolerance tolerance){
        return new RunCommand(
                () -> supersystem.setSupersystemState(setpoint.plus(new SupersystemState(turret.getAsDouble(), 0, 0, 0))), supersystem
        );
    }

    public Command goToSetpointWithManualAdjustment(SupersystemState setpoint, Constants.SupersystemTolerance tolerance){
        AtomicReference<Double> xOffset = new AtomicReference<>((double) 0);
        AtomicReference<Double> yOffset = new AtomicReference<>((double) 0);
        AtomicReference<Double> zOffset = new AtomicReference<>((double) 0);
        return new RunCommand(
                () -> {
                    //update offsets from inputs:
                    double newXOffset = xOffset.updateAndGet(v -> (v + x.getAsDouble()));
                    double newYOffset = yOffset.updateAndGet(v -> (v + y.getAsDouble()));
                    double newZOffset = zOffset.updateAndGet(v -> (v + z.getAsDouble()));

                    //calculate new position:
                    var position = kinematics(setpoint);
                    supersystem.setSupersystemPosition(
                            position.plus(new Translation3d(newXOffset, newYOffset, newZOffset))
                    );
                }, supersystem
        );
    }
}
