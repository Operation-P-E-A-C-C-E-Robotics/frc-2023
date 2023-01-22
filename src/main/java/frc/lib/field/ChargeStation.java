package frc.lib.field;

import frc.lib.util.Util;

/**
 * models the charge station for state space control
 */
public class ChargeStation {
    public static final double COUNTER_FORCE = 1; //the angular force exerted by the offset of the double hinge
    public static final double ROBOT_MASS = 1;
    public static final double INERTIA = 1;
    public static final double CHARGE_STATION_WIDTH = 0;
    public static final double ACCEPTABLE_ERROR = 2;
    public ChargeStation(){

    }
    //f = ma.
    //load * load distance = effort * effort distance
    /**
     * figure out the robot's position on the charge station
     * from the charge station's tilt
     * @param chargeStationPosition the position of the charge station degrees
     * @param chargeStationAcceleration the acceleration of the charge station degrees/s^2
     */
    public double calculateRobotPosition(double chargeStationPosition, double chargeStationAcceleration){
        boolean negative = false;
        double forceExertedByRobot, robotDistanceFromFulcrum;

        if(Util.inRange(chargeStationPosition, ACCEPTABLE_ERROR)) return 0;
        if(chargeStationPosition < 0){
             chargeStationAcceleration = -chargeStationAcceleration;
             negative = true;
        }
        forceExertedByRobot = ROBOT_MASS / (INERTIA * chargeStationAcceleration);
        robotDistanceFromFulcrum = COUNTER_FORCE / forceExertedByRobot;
        return negative ? -robotDistanceFromFulcrum - (CHARGE_STATION_WIDTH/2) : robotDistanceFromFulcrum + (CHARGE_STATION_WIDTH/2);
    }

    public static void main(String[] args){
        ChargeStation test = new ChargeStation();
        System.out.println(test.calculateRobotPosition(4, -1));
    }
}
