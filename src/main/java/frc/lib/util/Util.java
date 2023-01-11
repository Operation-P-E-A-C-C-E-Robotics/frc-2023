package frc.lib.util;

import java.util.ArrayList;
import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {

    public static final double kEpsilon = 1e-12;

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
        return epsilonEquals(a, b, kEpsilon);
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
     * @param ar
     * @param newv
     * @return
     */
    public static double[] shiftLeft(double[] ar, double newv){
        int i = 0;
        for (;i < ar.length - 1; i++){
            ar[i] = ar[i + 1];
        }
        ar[i] = newv;
        return ar;
    }
}