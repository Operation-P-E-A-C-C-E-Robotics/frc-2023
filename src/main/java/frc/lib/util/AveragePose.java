package frc.lib.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class AveragePose {
    SlewRateLimiter xFilter = new SlewRateLimiter(0.5);
    SlewRateLimiter yFilter = new SlewRateLimiter(0.5);
    SlewRateLimiter zFilter = new SlewRateLimiter(0.5);
    SlewRateLimiter pitchFilter = new SlewRateLimiter(0.5);
    SlewRateLimiter yawFilter = new SlewRateLimiter(0.5);
    SlewRateLimiter rollFilter = new SlewRateLimiter(0.5);
    Pose3d prev;
    public static double RESET_THRESHOLD = 0.1;

    private double calculateX(double input) {
        if (Double.isNaN(input)) {
            return input;
        }
        return xFilter.calculate(input);
    }
    
    private double calculateY(double input) {
        if (Double.isNaN(input)) {
            return input;
        }

        return yFilter.calculate(input);
    }
    
    private double calculateZ(double input) {
        if (Double.isNaN(input)) {
            return input;
        }

        return zFilter.calculate(input);
    }

    private double calculatePitch(double input) {
        if (Double.isNaN(input)) {
            return input;
        }

        return pitchFilter.calculate(input);
    }

    private double calculateYaw(double input) {
        if (Double.isNaN(input)) {
            return input;
        }
        return yawFilter.calculate(input);
    }

    private double calculateRoll(double input) {
        if (Double.isNaN(input)) {
           return input;
        }

        return rollFilter.calculate(input);
    }


    public Pose3d calculate(Pose3d unfilteredPose3d) {
        if(prev == null){
            reset(unfilteredPose3d);
            return unfilteredPose3d;
        }
        var delta = prev.minus(unfilteredPose3d);
        if(Math.max(Math.abs(delta.getX()), Math.abs(delta.getY())) > RESET_THRESHOLD){
            reset(unfilteredPose3d);
            return unfilteredPose3d;
        }

        prev = unfilteredPose3d;
        return new Pose3d(
            calculateX(unfilteredPose3d.getTranslation().getX()),
            calculateY(unfilteredPose3d.getTranslation().getY()),
            calculateZ(unfilteredPose3d.getTranslation().getZ()),
            new Rotation3d(
                calculateRoll(unfilteredPose3d.getRotation().getX()),
                calculatePitch(unfilteredPose3d.getRotation().getY()),
                calculateYaw(unfilteredPose3d.getRotation().getZ())
            )
        );
    }

    public void reset(Pose3d position) {
        xFilter.reset(position.getX());
        yFilter.reset(position.getY());
        zFilter.reset(position.getZ());
        rollFilter.reset(position.getRotation().getX());
        pitchFilter.reset(position.getRotation().getY());
        yawFilter.reset(position.getRotation().getZ());
    }

}
