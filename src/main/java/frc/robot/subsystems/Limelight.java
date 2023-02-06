package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.sensors.LimelightHelper;
import frc.lib.safety.Value;

public class Limelight extends LimelightHelper {
    private static final double CONE_WIDTH = 0.21,
                                CONE_HEIGHT = 0.32,
                                CUBE_WIDTH = 0.195;

    @SuppressWarnings("unused")
    private static final int APRILTAG_PIPELINE = 0,
                             CONE_PIPELINE = 1,
                             CUBE_PIPELINE = 2;

    public Limelight(String networktablesName) {
        super(networktablesName);
    }

    public Value<Translation2d> getConePoseRelativeToCamera(){
        setPipeline(CONE_PIPELINE);
        var distanceValue = getDistance(CONE_HEIGHT, CONE_WIDTH);
        var angleValue = getFilteredX();
        if(!Value.valid(distanceValue, angleValue)) return Value.notAvailable();

        var distance = distanceValue.get(0.0);
        var angle = Units.degreesToRadians(-angleValue.get(0.0));
        return new Value<>(
                new Translation2d(
                        distance * Math.cos(angle),
                        distance * Math.sin(angle)
                )
        );
    }

    public Value<Translation2d> getCubePoseRelativeToCamera(){
        setPipeline(CUBE_PIPELINE);
        var distanceValue = getDistance(CUBE_WIDTH, CUBE_WIDTH);
        var angleValue = getFilteredX();
        if(!Value.valid(distanceValue, angleValue)) return Value.notAvailable();

        var distance = distanceValue.get(0.0);
        var angle = Units.degreesToRadians(-angleValue.get(0.0));
        return new Value<>(
                new Translation2d(
                        distance * Math.cos(angle),
                        distance * Math.sin(angle)
                )
        );
    }
}
