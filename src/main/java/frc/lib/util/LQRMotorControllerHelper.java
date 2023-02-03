package frc.lib.util;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;

public class LQRMotorControllerHelper {
    private final LinearSystem plant;
    private final KalmanFilter observer;
    private final LinearQuadraticRegulator linearQuadraticRegulator;
    private final LinearSystemLoop loop;
    private final double maxVoltage;

    public LQRMotorControllerHelper(DCMotor motor,
                                    double inertia,
                                    double gearing,
                                    double cpr,
                                    double maxAngularVelocity,
                                    double maxAngularAcceleration,
                                    double kalmanModelAccuracyPosition,
                                    double kalmanModelAccuracyVelocity,
                                    double kalmanSensorAccuracyPosition,
                                    double kalmanSensorAccuracyVelocity,
                                    double lqrPositionTolerance,
                                    double lqrVelocityTolerance,
                                    double lqrControlEffortTolerance,
                                    double maxVoltage,
                                    double dt) {
        plant = LinearSystemId.createDCMotorSystem(
                motor,
                inertia,
                gearing
        );
        observer = new KalmanFilter(
                Nat.N2(),
                Nat.N2(),
                plant,
                VecBuilder.fill(kalmanModelAccuracyPosition, kalmanModelAccuracyVelocity),
                VecBuilder.fill(kalmanSensorAccuracyPosition, kalmanSensorAccuracyVelocity),
                dt
        );
        linearQuadraticRegulator = new LinearQuadraticRegulator(
                plant,
                VecBuilder.fill(lqrPositionTolerance, lqrVelocityTolerance),
                VecBuilder.fill(lqrControlEffortTolerance),
                dt
        );
        this.maxVoltage = maxVoltage;
    }
    public LinearSystemLoop<N2, N1, N2> getLoop() {
        return new LinearSystemLoop<>(
                plant,
                linearQuadraticRegulator,
                observer,
                maxVoltage,
                dt
        );
    }
}
