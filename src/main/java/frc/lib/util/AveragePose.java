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
    // SlewRateLimiter totalFilter = SlewRateLimiter.movingAverage(6);
    
    public double calculateX(double input) {
        if (Double.isNaN(input)) {
            reset();
        }
        return xFilter.calculate(input);
    }
    
    public double calculateY(double input) {
        if (Double.isNaN(input)) {
            reset();
        }

        return yFilter.calculate(input);
    }
    
    public double calculateZ(double input) {
        if (Double.isNaN(input)) {
            reset();
        }

        return zFilter.calculate(input);
    }

    public double calculatePitch(double input) {
        if (Double.isNaN(input)) {
            reset();
        }

        return pitchFilter.calculate(input);
    }

    public double calculateYaw(double input) {
        if (Double.isNaN(input)) {
            reset();
        }
        return yawFilter.calculate(input);
    }

    public double calculateRoll(double input) {
        if (Double.isNaN(input)) {
           reset();
        }

        return rollFilter.calculate(input);
    }


    // public SlewRateLimiter getFilter() {
    //     return totalFilter;
    // }

    public Pose3d calculate(Pose3d unfilteredPose3d) {
        Pose3d smoothedPose = new Pose3d(
            calculateX(unfilteredPose3d.getTranslation().getX()), 
            calculateY(unfilteredPose3d.getTranslation().getY()), 
            calculateZ(unfilteredPose3d.getTranslation().getZ()), 
            new Rotation3d(
                calculatePitch(unfilteredPose3d.getRotation().getX()), 
                calculateYaw(unfilteredPose3d.getRotation().getY()), 
                calculateRoll(unfilteredPose3d.getRotation().getZ())
            )
        );
        // System.out.println("=====Incoming Data=====");
        // System.out.println(unfilteredPose3d);
        // System.out.println("=====Outgoing Data=====");
        // System.out.println(smoothedPose);
        // System.out.println("========================");
        return smoothedPose;
    }

    public void reset() {
        xFilter.reset(0);
        yFilter.reset(0);
        zFilter.reset(0);
        pitchFilter.reset(0);
        yawFilter.reset(0);
        rollFilter.reset(0);
    }

}
