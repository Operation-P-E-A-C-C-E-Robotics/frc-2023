package frc.lib.geometry;

import edu.wpi.first.math.util.Units;

public class Distance {
    private final double meters;
    public Distance(double meters){
        this.meters = meters;
    }

    public double getMeters(){
        return meters;
    }

    public double getFeet(){
        return Units.metersToFeet(meters);
    }

    public double getInches(){
        return Units.metersToInches(meters);
    }

    public double getCentimeters(){
        return meters * 100;
    }

    public double getYards(){
        return Units.metersToFeet(meters) / 3;
    }

    public static Distance fromMeters(double meters){
        return new Distance(meters);
    }

    public static Distance fromFeet(double feet){
        return new Distance(Units.feetToMeters(feet));
    }

    public static Distance fromInches(double inches){
        return new Distance(Units.inchesToMeters(inches));
    }

    public static Distance fromCentimeters(double centimeters){
        return new Distance(centimeters / 100);
    }

    public static Distance fromYards(double yards){
        return new Distance(Units.feetToMeters(yards * 3));
    }

    public static Distance fromFeetAndInches(double feet, double inches){
        return new Distance(Units.feetToMeters(feet) + Units.inchesToMeters(inches));
    }
}
