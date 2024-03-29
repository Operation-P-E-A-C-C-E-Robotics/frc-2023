package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.lib.util.Util;
import frc.robot.subsystems.Supersystem;

import static frc.robot.Constants.Kinematics.PIVOT_HEIGHT;

public class Kinematics {
    SupersystemState state;
    Translation3d liftPosition;
    EndEffectorPosition endEffectorPosition;

    private final Supersystem supersystem;

    /**
     * handles kinematics of the supersystem
     * NOTE TO FUTURE PROGRAMMERS: Since I foresee getSupersystemState/Position
     * getting called a lot, I cache their values, so we don't recalculate multiple times per loop
     * but, each loop we must call reset() to clear the cache.
     */
    public Kinematics(Supersystem supersystem){
        this.supersystem = supersystem;
    }

    /**
     * THIS FUNCTION MUST BE CALLED EVERY LOOP
     * resets caching of values to prevent unnecesarry computation
     */
    public void reset(){
        state = null;
        liftPosition = null;
        endEffectorPosition = null;
    } //TODO make sure calling every loop

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
     */
    public Translation3d getSupersystemPosition(){
        if (liftPosition == null) liftPosition = kinematics(getSupersystemState());
        return liftPosition;
    }

    /**
     * get the position of the wrist, both the end of the wrist,
     * and the point where gamepieces are centered for
     * placing.
     * @return {@link EndEffectorPosition}
     */
    public EndEffectorPosition getEndEffectorPosition(){
        if (endEffectorPosition == null) endEffectorPosition = wristInverseKinematics(
            getSupersystemPosition(),
            getSupersystemState().getTurretAngle(),
            getSupersystemState().getWristAngle()
        );
        return endEffectorPosition;
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
    public static SupersystemState inverseKinematics(Translation3d pose, double wristAngle){
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

    /**
     * Convert from x/y/z coordinates of the end of the wrist
     */
    public static SupersystemState inverseKinematicsFromEndEffector(Translation3d pose, double wristAngle){
        double turret;

        if(pose.getX() == 0 && pose.getY() == 0){
            turret = 0;
        } else {
            //magic math from https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/12%3A_Vectors_in_Space/12.07%3A_Cylindrical_and_Spherical_Coordinates
            turret = Math.atan(pose.getY()/pose.getX());
        }

        Translation3d wristOffset = wristOffset(turret, wristAngle).getEndPosition();
        Translation3d poseWithoutWrist = pose.minus(wristOffset);

        return inverseKinematics(poseWithoutWrist, wristAngle);
    }

    /**
     * Convert from x/y/z coordinates of the center of the wrist
     * @param pose the target translation
     * @param wristAngle the angle of the wrist
     * @return the SupersystemState to reach the target translation.
     */
    public static SupersystemState inverseKinematicsFromPlacePoint(Translation3d pose, double wristAngle){
        double turret;

        if(pose.getX() == 0 && pose.getY() == 0){
            turret = 0;
        } else {
            //magic math from https://math.libretexts.org/Bookshelves/Calculus/Book%3A_Calculus_(OpenStax)/12%3A_Vectors_in_Space/12.07%3A_Cylindrical_and_Spherical_Coordinates
            turret = Math.atan(pose.getY()/pose.getX());
        }

        Translation3d wristOffset = wristOffset(turret, wristAngle).getMidPosition();
        Translation3d poseWithoutWrist = pose.minus(wristOffset);

        return inverseKinematics(poseWithoutWrist, wristAngle);
    }

    public static SupersystemState inverseKinematicsFromPlacePoint2(SupersystemState inverseKinematics, double wristAngle){
        var pivot = inverseKinematics.getPivotAngle();
        var extension = inverseKinematics.getArmExtension();
        var wristInside = Math.PI - wristAngle;
        
        var sin = Math.sin(wristInside) / extension;
        var deltaPivot = Math.asin(sin*WRIST_MID_LENGTH);
        
        var thirdAngle = Math.PI - wristInside - deltaPivot;
        var newExtension = (1/sin)*Math.sin(thirdAngle);

        return new SupersystemState(inverseKinematics.getTurretAngle(), pivot + deltaPivot, newExtension, wristAngle);
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
    public static Translation3d kinematics(SupersystemState state){
        double x, y, z;
        double arm = state.getArmExtension(),
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

        return new Translation3d(x, y, z);
    }

    public static final double WRIST_LENGTH = 0.2,
                              WRIST_MID_LENGTH = -0.4; //todo move to constants
    /**
     * Convert from supersystem joint positions to location of the wrist
     * @param liftState the state of the lift
     * @return the position of the wrist with the lift in that state.
     */
    public static EndEffectorPosition wristKinematics(SupersystemState liftState){
        Translation3d liftPosition = kinematics(liftState);

        return wristInverseKinematics(liftPosition, liftState.getTurretAngle(), liftState.getWristAngle());
    }

    /**
     * Convert from x/y/z position of the end of the wrist to the position of
     * the end of the arm, using the current turret angle and wrist angle
     * @param liftPosition the position of the end of the wrist
     * @param turretAngle the angle of the turret
     * @param wristAngle the angle of the wrist
     * @return the position of the end of the arm
     */
    public static EndEffectorPosition wristInverseKinematics(Translation3d liftPosition, double turretAngle, double wristAngle){
        EndEffectorPosition offset = wristOffset(turretAngle, wristAngle);
        return new EndEffectorPosition(
            liftPosition.plus(offset.getEndPosition()),
            liftPosition.plus(offset.getMidPosition())
        );
    }

    /**
     * get the contribution of the wrist to the end effector position
     * @param turretAngle the angle of the turret
     * @param wristAngle the angle of the wrist
     * @return the position of the end of the wrist relative to the end of the arm
     */
    public static EndEffectorPosition wristOffset(double turretAngle, double wristAngle){
        double endX, endY, endZ;
        double midX, midY, midZ;
        double x, y, z;

        // x = Math.sin(wristAngle) * Math.cos(turretAngle);
        // y = Math.sin(wristAngle) * Math.sin(turretAngle);
        // z = Math.cos(wristAngle);

        //assume max length:
        x = Math.cos(turretAngle);
        y = Math.sin(turretAngle);
        z = 0;

        endX = WRIST_LENGTH * x;
        endY = WRIST_LENGTH * y;
        endZ = WRIST_LENGTH * z;

        midX = WRIST_MID_LENGTH * x;
        midY = WRIST_MID_LENGTH * y;
        midZ = WRIST_MID_LENGTH * z;
        return new EndEffectorPosition(
            new Translation3d(endX, endY, endZ),
            new Translation3d(midX, midY, midZ)
        );
    }

    /**
     * Optimize the supersystem state to minimize motions of the joints.
     * The output of inverse kinematics only allows the turret to rotate
     * -90 to 90 degrees before flipping. This allows us to use the full travel
     * of the turret, by flipping the state only once the turret is at the
     * end of its travel (-270 to 270 degrees).
     * @param target the target state
     * @param current the current state
     * @return the optimized state
     */
    public static SupersystemState optimize(SupersystemState target, SupersystemState current){
        // figure out what multiple of 180 degrees to add to get us closest to the current state:
        double turret = target.getTurretAngle();
        double pivot = target.getPivotAngle();
        int flips = (int) Math.round((current.getTurretAngle() - turret) / (Math.PI));

        // flip flips number of times:
        for(int i = 0; i < Math.abs(flips); i++){
            turret += Math.PI * Math.signum(flips);
            pivot = -pivot;
        }
        // if we move too far, return the original state:
        if(!Util.inRange(turret, Units.degreesToRadians(Constants.Turret.MAX_ANGLE_RAD))) return target;

        //return whichever state is closer to the current state:
        var flippedState = new SupersystemState(turret, pivot, target.getArmExtension(), target.getWristAngle());
        return timeToMove(flippedState, current) < timeToMove(target, current) ? flippedState : target;
    }

    private static final double TURRET_VELOCITY = 0.5;
    private static final double PIVOT_VELOCITY = 0.5;
    private static double timeToMove(SupersystemState target, SupersystemState current){
        var deltaTurret = Math.abs(target.getTurretAngle() - current.getTurretAngle());
        var deltaPivot = Math.abs(target.getPivotAngle() - current.getPivotAngle());
        return Math.max(deltaTurret / TURRET_VELOCITY, deltaPivot / PIVOT_VELOCITY);
    }

    public static void main(String args[]){
        //test new inverse kinematics from place point:
        var targetXYZ = new Translation3d(0, 0, 1);
        var targetState = inverseKinematics(targetXYZ, 0);
        System.out.println("target state: " + targetState);
        var check = kinematics(targetState);
        System.out.println("check: " + check);
        var placePoint = inverseKinematicsFromPlacePoint2(targetState, Math.PI/2);
        System.out.println("place point: " + placePoint);
        var check2 = kinematics(placePoint);
        System.out.println("check: " + check2);

    }

    public static class EndEffectorPosition {
        private final Translation3d endPosition, midPosition;
        public EndEffectorPosition(Translation3d endPosition, Translation3d midPosition){
            this.endPosition = endPosition;
            this.midPosition = midPosition;
        }
        public Translation3d getEndPosition(){
            return endPosition;
        }
        public Translation3d getMidPosition(){
            return midPosition;
        }
        public String toString(){
            return "wrist end: " + endPosition + " place point: " + midPosition;
        }
    }

    public static class SupersystemState{
        private final double liftExtension;
        private final double turretAngle, pivotAngle;
        private final double wristAngle;

        /**
         * class to hold a target state of the gamepiece handler system
         * @param turretAngle angle of the turret - forward = 0 - counter-clockwise positive
         * @param pivotAngle angle of the pivot - up = 0 - positive tips forward
         * @param liftExtension total distance from pivot to intake in meters
         */
        public SupersystemState(double turretAngle, double pivotAngle, double liftExtension, double wristAngle){
            this.turretAngle = turretAngle;
            this.pivotAngle = pivotAngle;
            this.liftExtension = liftExtension;
            this.wristAngle = wristAngle;
        }
        public double getTurretAngle(){
            return turretAngle;
        }
        public double getPivotAngle(){
            return pivotAngle;
        }
        public double getArmExtension(){
            return liftExtension;
        }
        public double getWristAngle(){
            return wristAngle;
        }

        public SupersystemState plus(SupersystemState other){
            return new SupersystemState(
                    turretAngle + other.getTurretAngle(),
                    pivotAngle + other.getPivotAngle(),
                    liftExtension + other.getArmExtension(),
                    wristAngle + other.getWristAngle()
            );
        }
        public String toString(){
            return "turret: " + new Rotation2d(turretAngle).getDegrees() + " pivot: " + new Rotation2d(pivotAngle).getDegrees() + " arm: " + liftExtension;
        }
    }
}
