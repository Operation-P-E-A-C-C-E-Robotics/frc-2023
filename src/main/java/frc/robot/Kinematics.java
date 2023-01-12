package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Kinematics {
    public static final double PIVOT_HEIGHT = 0.3; //TODO how high the pivot is above the ground

    /**
     * Convert from an x/y/z position of the manipulator to the position of
     * the manipulator joints
     * Coordinates:
     *  x: front of the robot
     *  y: left-positive
     *  z: up
     *  turret rotation: 0: forward, counter-clockwise positive
     *  pivot rotation: 0: up, positive tips forward
     * if the target is straight up, the turret will be set to zero.
     * @param pose the target pose
     * @return the handlerstate to reach the target pose.
     */
    public static HandlerState kinematics(HandlerPosition pose){
        double extension, turret, pivot,
            z = pose.getZ() - PIVOT_HEIGHT; //account for height of pivot

        // calculate arm extension dist (similar to pythagorean theorem,
        // 3D distance formula)
        extension = Math.sqrt(
            pose.getX()*pose.getX() +
            pose.getY()*pose.getY() +
            z*z
        );

        //account for when x and y = 0 making turret and pivot undefined - set both to 0.
        if(pose.getX() == 0 && pose.getY() == 0){
            turret = 0;
            pivot = 0;
        } else {
            //magic math from https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/12%3A_Vectors_in_Space/12.07%3A_Cylindrical_and_Spherical_Coordinates
            turret = Math.atan(pose.getY()/pose.getX());
            pivot = Math.acos(z/extension);
        }

        //Invert the pivot to reach the back of the robot
        //For whatever reason, the turret inverts correctly when X is negative, so this
        //is all we have to do.
        if(pose.getX() < 0) pivot = -pivot;
        return new HandlerState(turret, pivot, extension);
    }

    /**
     * Convert from manipulator joint positions to the location of
     * the intake x/y/z
     * Coordinates:
     *  x: front of the robot
     *  y: left-positive
     *  z: up
     *  turret rotation: 0: forward, counter-clockwise positive
     *  pivot rotation: 0: up, positive tips forward
     * @param state A state of the manipulator
     * @return the position of the intake when the manipulator is in that state
     */
    public static HandlerPosition inverseKinematics(HandlerState state){
        double x, y, z;
        double arm = state.getArmLength(),
                pivot = state.getPivotAngle(),
                turret = state.getTurretAngle();

        x = arm * Math.sin(pivot) * Math.cos(turret);
        y = arm * Math.sin(pivot) * Math.sin(turret);
        //weird sht but it works
        if(pivot == 0) {
            z = arm - PIVOT_HEIGHT;
        } else {
            z = arm * Math.cos(pivot) + PIVOT_HEIGHT;
        }

        return new HandlerPosition(x, y, z);
    }

    //testing
    public static void main(String args[]){
        HandlerPosition testPose = new HandlerPosition(0, 0, 0);
        System.out.println(testPose);
        System.out.println(Kinematics.kinematics(testPose));
        System.out.println(Kinematics.inverseKinematics(Kinematics.kinematics(testPose)));
    }

    public static class HandlerState{
        private double armLength;
        private double turretAngle, pivotAngle;

        /**
         * class to hold a target state of the gamepiece handler system
         * @param turretAngle angle of the turret - forward = 0 - counter-clockwise positive
         * @param pivotAngle angle of the pivot - up = 0 - positive tips forward
         * @param armLength total distance from pivot to intake in meters
         */
        public HandlerState(double turretAngle, double pivotAngle, double armLength){
            this.turretAngle = turretAngle;
            this.pivotAngle = pivotAngle;
            this.armLength = armLength;
        }
        public double getTurretAngle(){
            return turretAngle;
        }
        public double getPivotAngle(){
            return pivotAngle;
        }
        public double getArmLength(){
            return armLength;
        }
        public String toString(){
            return "turret: " + new Rotation2d(turretAngle).getDegrees() + " pivot: " + new Rotation2d(pivotAngle).getDegrees() + " arm: " + armLength;
        }
    }

    public static class HandlerPosition{
        private double x, y, z;
        /**
         * A class to hold the position of the intake, relative to the robot
         * @param x positive towards the front of the robot
         * @param y positive to the left of the robot
         * @param z positive up
         */
        public HandlerPosition(double x, double y, double z){
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public double getX(){
            return x;
        }
        public double getY(){
            return y;
        }
        public double getZ(){
            return z;
        }
        public static HandlerPosition fromTranslation(Translation2d translation, double height){
            return new HandlerPosition(translation.getX(), translation.getY(), height);
        }
        public String toString(){
            return "x: " + x + " y: " + y + " z: " + z;
        }
    }
}
