package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.sensors.LimelightHelper;
import frc.lib.safety.Value;

public class Limelight extends LimelightHelper {
    private static final double CONE_WIDTH = 0.21,
                                CONE_HEIGHT = 0.32,
                                CUBE_SIDE = 0.195;

    @SuppressWarnings("unused")
    private static final int APRILTAG_PIPELINE = 0,
                             CONE_PIPELINE = 1,
                             CUBE_PIPELINE = 2;

    public Limelight(String networktablesName) {
        super(networktablesName);
    }

    public Value<Translation2d> getConePoseRelativeToCamera(){
        setPipeline(CONE_PIPELINE);
        return getPose(CONE_WIDTH, CONE_HEIGHT);
    }

    public Value<Translation2d> getCubePoseRelativeToCamera(){
        if(getPipeline() != CUBE_PIPELINE) {
            setPipeline(CUBE_PIPELINE);
            return Value.none();
        }
        return getPose(CUBE_SIDE, CUBE_SIDE);
    }

    private Value<Translation2d> getPose(double width, double height){
        var distanceValue = getDistance(height, width);
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
