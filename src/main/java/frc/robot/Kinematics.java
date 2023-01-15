package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Supersystem;

import static frc.robot.Constants.Kinematics.*;

public class Kinematics {
    SupersystemState state;
    Position liftPosition;
    WristPosition wristPosition;

    private Supersystem supersystem;

    /**
     * handles kinematics of the supersystem
     * @param supersystem
     */
    public Kinematics(Supersystem supersystem){
        this.supersystem = supersystem;
    }

    /**
     * THIS FUNCTION MUST BE CALLED EVERY LOOP
     * resets caching of values to prevent unnecesarry computation
     */
    public void reset(){
        liftPosition = null;
        wristPosition = null;
    }

    /**
     * get the position of all the supersystem
     * joints, from the {@link Supersystem}
     * @return {@link SupersystemState}
     */
    public SupersystemState getSupersystemState(){
        if (state == null) state = supersystem.getSupersystemState();
        return state;
    }

    /**
     * get the position of the lift (end of the arm)
     * as xyz coordinates.
     * x - toward the front of the robot
     * y - toward the left of the robot
     * z - up
     * @return
     */
    public Position getSupersystemPosition(){
        if (liftPosition == null) liftPosition = kinematics(getSupersystemState());
        return liftPosition;
    }

    /**
     * get the position of the wrist, both the end of the wrist,
     * as well as the point where gamepieces are centered for
     * placing.
     * @return {@link WristPosition}
     */
    public WristPosition getWristPosition(){
        if (wristPosition == null) wristPosition = wristInverseKinematics(
            getSupersystemPosition(),
            getSupersystemState().getTurretAngle(),
            getSupersystemState().getWristAngle()
        );
        return wristPosition;
    }

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
     * @return the LiftState to reach the target pose.
     */
    public static SupersystemState inverseKinematics(Position pose, double wristAngle){
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

        return new SupersystemState(turret, pivot, extension, wristAngle);
    }

    public static SupersystemState inverseKinematicsFromWristEnd(Position pose, double wristAngle){
        double turret;

        turret = Math.atan(pose.getY()/pose.getX());

        if(pose.getX() == 0 && pose.getY() == 0){
            turret = 0;
        } else {
            //magic math from https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/12%3A_Vectors_in_Space/12.07%3A_Cylindrical_and_Spherical_Coordinates
            turret = Math.atan(pose.getY()/pose.getX());
        }

        Position wristOffset = wristOffset(turret, wristAngle).getEndPosition();
        Position poseWithoutWrist = pose.subtract(wristOffset);

        return inverseKinematics(poseWithoutWrist, wristAngle);
    }

    public static SupersystemState inverseKinematicsFromWristPlacePoint(Position pose, double wristAngle){
        double turret;

        turret = Math.atan(pose.getY()/pose.getX());

        if(pose.getX() == 0 && pose.getY() == 0){
            turret = 0;
        } else {
            //magic math from https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/12%3A_Vectors_in_Space/12.07%3A_Cylindrical_and_Spherical_Coordinates
            turret = Math.atan(pose.getY()/pose.getX());
        }

        Position wristOffset = wristOffset(turret, wristAngle).getMidPosition();
        Position poseWithoutWrist = pose.subtract(wristOffset);

        return inverseKinematics(poseWithoutWrist, wristAngle);
    }

    /**
     * Convert from manipulator joint positions to the location of
     * the end of the lift x/y/z
     * Coordinates:
     *  x: front of the robot
     *  y: left-positive
     *  z: up
     *  turret rotation: 0: forward, counter-clockwise positive
     *  pivot rotation: 0: up, positive tips forward
     * @param state A state of the manipulator
     * @return the position of the intake when the manipulator is in that state
     */
    public static Position kinematics(SupersystemState state){
        double x, y, z;
        double arm = state.getLiftExtension(),
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

        return new Position(x, y, z);
    }

    public static final double WRIST_LENGTH = 0.2,
                              WRIST_MID_LENGTH = 0.1; //todo move to constants
    /**
     * Convert from supersystem joint positions to location of the wrist
     * @param liftState the state of the lift
     * @return the position of the wrist with the lift in that state.
     */
    public static WristPosition wristKinematics(SupersystemState liftState){
        Position liftPosition = kinematics(liftState);

        return wristInverseKinematics(liftPosition, liftState.getTurretAngle(), liftState.getWristAngle());
    }
    public static WristPosition wristInverseKinematics(Position liftPosition, double turretAngle, double wristAngle){
        WristPosition offset = wristOffset(turretAngle, wristAngle);
        return new WristPosition(
            liftPosition.add(offset.getEndPosition()),
            liftPosition.add(offset.getMidPosition())
        );
    }
    public static WristPosition wristOffset(double turretAngle, double wristAngle){
        double endX, endY, endZ;
        double midX, midY, midZ;
        double x, y, z;

        x = Math.sin(wristAngle) * Math.cos(turretAngle);
        y = Math.sin(wristAngle) * Math.sin(turretAngle);
        z = Math.cos(wristAngle);

        endX = WRIST_LENGTH * x;
        endY = WRIST_LENGTH * y;
        endZ = WRIST_LENGTH * z;

        midX = WRIST_MID_LENGTH * x;
        midY = WRIST_MID_LENGTH * y;
        midZ = WRIST_MID_LENGTH * z;
        return new WristPosition(
            new Position(endX, endY, endZ),
            new Position(midX, midY, midZ)
        );
    }

    //testing
    public static void main(String args[]){
        Position testPose = new Position(1, 0, 0);
        System.out.println(testPose);
        System.out.println(Kinematics.inverseKinematics(testPose, 0));
        System.out.println(Kinematics.wristKinematics(Kinematics.inverseKinematics(testPose, Rotation2d.fromDegrees(-90).getRadians())));
        System.out.println(Kinematics.inverseKinematicsFromWristEnd(
            Kinematics.wristKinematics(
                Kinematics.inverseKinematics(
                    testPose, Rotation2d.fromDegrees(0).getRadians()
                )
            ).getEndPosition(), Rotation2d.fromDegrees(0).getRadians()
        ));
    }

    public static class WristPosition{
        private Position endPosition, midPosition;
        public WristPosition(Position endPosition, Position midPosition){
            this.endPosition = endPosition;
            this.midPosition = midPosition;
        }
        public Position getEndPosition(){
            return endPosition;
        }
        public Position getMidPosition(){
            return midPosition;
        }
        public String toString(){
            return "wrist end: " + endPosition + " place point: " + midPosition;
        }
    }

    public static class SupersystemState{
        private double armLength;
        private double turretAngle, pivotAngle;
        private double wristAngle;

        /**
         * class to hold a target state of the gamepiece handler system
         * @param turretAngle angle of the turret - forward = 0 - counter-clockwise positive
         * @param pivotAngle angle of the pivot - up = 0 - positive tips forward
         * @param armLength total distance from pivot to intake in meters
         */
        public SupersystemState(double turretAngle, double pivotAngle, double armLength, double wristAngle){
            this.turretAngle = turretAngle;
            this.pivotAngle = pivotAngle;
            this.armLength = armLength;
            this.wristAngle = wristAngle;
        }
        public double getTurretAngle(){
            return turretAngle;
        }
        public double getPivotAngle(){
            return pivotAngle;
        }
        public double getLiftExtension(){
            return armLength;
        }
        public double getWristAngle(){
            return wristAngle;
        }
        public String toString(){
            return "turret: " + new Rotation2d(turretAngle).getDegrees() + " pivot: " + new Rotation2d(pivotAngle).getDegrees() + " arm: " + armLength;
        }
    }

    public static class Position{
        private double x, y, z;
        /**
         * A class to hold the position of the lift, relative to the robot
         * @param x positive towards the front of the robot
         * @param y positive to the left of the robot
         * @param z positive up
         */
        public Position(double x, double y, double z){
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
        public static Position fromTranslation(Translation2d translation, double height){
            return new Position(translation.getX(), translation.getY(), height);
        }
        public String toString(){
            return "x: " + x + " y: " + y + " z: " + z;
        }
        public Position add(double x, double y, double z){
            return new Position(this.x + x, this.y + y, this.z + z);
        }
        public Position add(Position other){
            return new Position(this.x + other.getX(), this.y + other.getY(), this.z + other.getZ());
        }
        public Position subtract(Position other){
            return new Position(this.x - other.getX(), this.y - other.getY(), this.z - other.getZ());
        }
    }
}
