package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.subsystems.Supersystem;

import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

import static frc.robot.Kinematics.*;

public class Setpoints {

    public static SupersystemState zero = new SupersystemState(0, 0, 0, 0);
    public static SupersystemState ninetyPivot = new SupersystemState(0, -Math.PI/2, 0, 0);

    public static SupersystemState placeLow = new SupersystemState(0, -1.5, 0, -1);

    public static SupersystemState placeMidCube = new SupersystemState(0, 0, 0, 0);
    public static SupersystemState placeHighCube = new SupersystemState(0, 0.5, 1.5, 10);

    public static SupersystemState placeMidCone = new SupersystemState(0, -1.0754, 0.7051, 0.5);
    public static SupersystemState placeHighCone = new SupersystemState(0, -0.99, 1.28, 0.5);

    public static SupersystemState intakeFloor = new SupersystemState(0, Units.degreesToRadians(120), 0.93, -0.5);
    public static SupersystemState intakeDoubleSubstation = new SupersystemState(0, -0.8, 0.69, Units.degreesToRadians(-90));
    public static SupersystemState wristFlipOffset = new SupersystemState(0, 0, 0, -0.5);
    private final Supersystem supersystem;
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier z;
    private final DoubleSupplier turret;

    public Setpoints(Supersystem supersystem, DoubleSupplier x, DoubleSupplier y, DoubleSupplier z,
            DoubleSupplier turret) {
        this.supersystem = supersystem;
        this.x = x;
        this.y = y;
        this.z = z;
        this.turret = turret;
    }

    public Command goToSetpoint(SupersystemState setpoint, Constants.SupersystemTolerance tolerance) {
        AtomicReference<Double> turretOffset = new AtomicReference<>(0.0);
        return new RunCommand(
                () -> supersystem.setSupersystemState(setpoint
                        .plus(new SupersystemState(turretOffset.updateAndGet(v -> v + turret.getAsDouble()), 0, 0, 0))),
                supersystem.getRequirements());
    }

    public Command goToSetpoint(SupersystemState setpoint) {
        return goToSetpoint(setpoint, Constants.SupersystemTolerance.DEFAULT);
    }

    public Command goToSetpoint(SupersystemState setpoint, Constants.SupersystemTolerance tolerance,
            boolean endOnTarget) {
        if (!endOnTarget)
            return goToSetpoint(setpoint, tolerance);
        AtomicReference<Double> turretOffset = new AtomicReference<>(0.0);
        return new RunCommand(
                () -> supersystem.setSupersystemState(setpoint
                        .plus(new SupersystemState(turretOffset.updateAndGet(v -> v + turret.getAsDouble()), 0, 0, 0))),
                supersystem.getRequirements()).until(() -> supersystem.withinTolerance(tolerance));
    }

    public Command goToPlace(Automations.PlaceLevel level, boolean cone) {
        var tolerance = Constants.SupersystemTolerance.forLevel(level);
        return switch (level) {
            case LOW -> goToSetpoint(placeLow, tolerance);
            case MID -> goToSetpoint(cone ? placeMidCone : placeMidCube, tolerance);
            case HIGH -> goToSetpoint(cone ? placeHighCone : placeHighCube, tolerance);
        };
    }

    public Command goToPlaceAndEnd(Automations.PlaceLevel level, boolean cone) {
        var tolerance = Constants.SupersystemTolerance.forLevel(level);
        return switch (level) {
            case LOW -> goToSetpoint(placeLow, tolerance, true);
            case MID -> goToSetpoint(cone ? placeMidCone : placeMidCube, tolerance, true);
            case HIGH -> goToSetpoint(cone ? placeHighCone : placeHighCube, tolerance, true);
        };
    }

    public Command zero(){
        return goToSetpoint(Setpoints.zero, Constants.SupersystemTolerance.DEFAULT, true);
    }

    public Command goToPlaceConeWithWristFlip(Automations.PlaceLevel level, boolean endOnTarget) {
        var setpoint = switch (level) {
            case LOW -> placeLow;
            case MID -> placeMidCone;
            case HIGH -> placeHighCone;
        };
        return goToSetpoint(setpoint.plus(wristFlipOffset), Constants.SupersystemTolerance.forLevel(level), true)
                .andThen(goToSetpoint(setpoint, Constants.SupersystemTolerance.forLevel(level), endOnTarget));
    }

    public Command goToSetpointWithManualAdjustment(SupersystemState setpoint,
            Constants.SupersystemTolerance tolerance) {
        AtomicReference<Double> xOffset = new AtomicReference<>((double) 0);
        AtomicReference<Double> yOffset = new AtomicReference<>((double) 0);
        AtomicReference<Double> zOffset = new AtomicReference<>((double) 0);
        return new RunCommand(
                () -> {
                    // update offsets from inputs:
                    double newXOffset = xOffset.updateAndGet(v -> (v + x.getAsDouble()));
                    double newYOffset = yOffset.updateAndGet(v -> (v + y.getAsDouble()));
                    double newZOffset = zOffset.updateAndGet(v -> (v + z.getAsDouble()));

                    // calculate new position:
                    var position = kinematics(setpoint);
                    supersystem.setSupersystemPosition(
                            position.plus(new Translation3d(newXOffset, newYOffset, newZOffset)));
                }, supersystem);
    }

    public Command goToPlaceWithManualAdjustment(Automations.PlaceLevel level, boolean cone) {
        var tolerance = Constants.SupersystemTolerance.forLevel(level);
        return switch (level) {
            case LOW -> goToSetpointWithManualAdjustment(placeLow, tolerance);
            case MID -> goToSetpointWithManualAdjustment(cone ? placeMidCone : placeMidCube, tolerance);
            case HIGH -> goToSetpointWithManualAdjustment(cone ? placeHighCone : placeHighCube, tolerance);
        };
    }
}
