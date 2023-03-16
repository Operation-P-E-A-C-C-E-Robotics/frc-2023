package frc.lib.geometry;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.field.FieldConstants;

import java.util.function.Supplier;

public class RectangularRegion2d {
    private final Translation2d corner1, corner2;

    public RectangularRegion2d(Translation2d corner1, Translation2d corner2){
        this.corner1 = corner1;
        this.corner2 = corner2;
    }

    public boolean contains(Translation2d point){
        return point.getX() >= Math.min(corner1.getX(), corner2.getX()) &&
                point.getX() <= Math.max(corner1.getX(), corner2.getX()) &&
                point.getY() >= Math.min(corner1.getY(), corner2.getY()) &&
                point.getY() <= Math.max(corner1.getY(), corner2.getY());
    }

    public Trigger getTrigger(Supplier<Translation2d> pointSupplier) {
        return new Trigger(() -> contains(pointSupplier.get()));
    }

    public Translation2d getCenter(){
        return new Translation2d(
                (corner1.getX() + corner2.getX()) / 2,
                (corner1.getY() + corner2.getY()) / 2
        );
    }

    public static RectangularRegion2d fromCenter(Translation2d center, double width, double height){
        return new RectangularRegion2d(
                new Translation2d(center.getX() - width / 2, center.getY() - height / 2),
                new Translation2d(center.getX() + width / 2, center.getY() + height / 2)
        );
    }

    public static RectangularRegion2d fieldFullWidth(double x1, double x2){
        double fieldWidth = FieldConstants.fieldWidth;
        return new RectangularRegion2d(
                new Translation2d(x1, 0),
                new Translation2d(x2, fieldWidth)
        );
    }
}
