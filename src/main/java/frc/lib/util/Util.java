package frc.lib.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;

import java.util.ArrayList;
import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {

    public static final double EPSILON = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static double interpolate(double a, double b, double x) {
        x = limit(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    public static List<Double> subList(List<Double> list, int start, int end){
        ArrayList<Double> newList = new ArrayList<>();
        for(int i = start; i <= end; i++){
            newList.add(list.get(i));
        }
        return newList;
    }

    public static boolean inRange(double in, double range){
        return (Math.abs(in) < Math.abs(range));
    }

    public static Double last(List<Double> ar, int i){
        return ar.get(ar.size() - 1 - i);
    }

    /**
     * index an array from the end
     * @param ar double array
     * @param i index from back of array
     * @return array element
     */
    public static double last(double[] ar, int i){
        return ar[ar.length - 1 - i];
    }

    /**
     * shift an element into the left side of a double array, moving all elements one step
     */
    public static double[] shiftLeft(double[] ar, double newv){
        int i = 0;
        for (;i < ar.length - 1; i++){
            ar[i] = ar[i + 1];
        }
        ar[i] = newv;
        return ar;
    }

    public static double handleDeadband(double joystickPosition, double deadband) {
        if(inRange(joystickPosition, deadband)) return 0;
        return joystickPosition;
    }

    public static Pose3d toPose3d(Translation2d translation) {
        return toPose3d(new Pose2d(translation, new Rotation2d()));
    }

    public static Pose3d toPose3d(Pose2d pose){
        return toPose3d(pose, 0,0,0);
    }

    public static Pose3d toPose3d(Pose2d pose, double z, double roll, double pitch){
        return new Pose3d(
                pose.getX(),
                pose.getY(),
                0,
                new Rotation3d(roll, pitch, pose.getRotation().getRadians())
        );
    }

    public static Pose2d toPose2d(Pose3d pose){
        return new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getZ()));
    }

    /**
     * convert a pose from one coordinate system to another, where the origin of
     * the local coordinate system is known
     * @param localOrigin the pose of the base of the local coordinate system relative to the global origin
     * @param localPoint the local pose to convert
     * @return the pose relative to the global origin
     */
    public static Pose3d localToGlobalPose(Pose3d localOrigin, Pose3d localPoint) {
        Translation3d rotatedTranslation = rotateBy(localPoint.getTranslation(), localOrigin.getRotation());
        Rotation3d rotatedRotation = localPoint.getRotation().plus(localOrigin.getRotation()); //TODO plus or mines

        return new Pose3d(
                localOrigin.getTranslation().plus(rotatedTranslation),
                rotatedRotation
        );
    }

    public static Pose3d globalToLocalPose(Pose3d localOrigin, Pose3d globalPoint) {
        Translation3d localTranslation = globalPoint.getTranslation().minus(localOrigin.getTranslation());
        Translation3d rotatedTranslation = rotateBy(localTranslation, new Rotation3d().minus(localOrigin.getRotation()));
        Rotation3d rotatedRotation = globalPoint.getRotation().minus(localOrigin.getRotation()); //TODO plis or munesz
        ADIS16470_IMU gyro = new ADIS16470_IMU();
        gyro.getXComplementaryAngle();
        return new Pose3d(
                rotatedTranslation,
                rotatedRotation
        );
    }

    //https://stackoverflow.com/questions/34050929/3d-point-rotation-algorithm
    public static Translation3d rotateBy(Translation3d point, Rotation3d rotation){
        double cosa = Math.cos(rotation.getZ());
        double sina = Math.sin(rotation.getZ());

        double cosb = Math.cos(rotation.getY());
        double sinb = Math.sin(rotation.getY());

        double cosc = Math.cos(rotation.getX());
        double sinc = Math.sin(rotation.getX());

        var Axx = cosa*cosb;
        var Axy = cosa*sinb*sinc - sina*cosc;
        var Axz = cosa*sinb*cosc + sina*sinc;

        var Ayx = sina*cosb;
        var Ayy = sina*sinb*sinc + cosa*cosc;
        var Ayz = sina*sinb*cosc - cosa*sinc;

        var Azx = -sinb;
        var Azy = cosb*sinc;
        var Azz = cosb*cosc;

        return new Translation3d(
                Axx*point.getX() + Axy*point.getY() + Axz*point.getZ(),
                Ayx*point.getX() + Ayy*point.getY() + Ayz*point.getZ(),
                Azx*point.getX() + Azy*point.getY() + Azz*point.getZ()
        );
    }

    public static void main(String args[]){
        var localOrigin = new Pose3d(
                1,0,0,
                new Rotation3d(0.4,0, Units.degreesToRadians(130))
        );
        var pt = new Pose3d(
                1,0,0,
                new Rotation3d(0,0,0)
        );
        System.out.println(globalToLocalPose(localOrigin,localToGlobalPose(
                localOrigin,
                pt
        )));
        System.out.println(localToGlobalPose(
                localOrigin,
                pt
        ));
    }
}