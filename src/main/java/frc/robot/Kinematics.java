package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
                              WRIST_MID_LENGTH = 0.1; //todo move to constants
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

        x = Math.sin(wristAngle) * Math.cos(turretAngle);
        y = Math.sin(wristAngle) * Math.sin(turretAngle);
        z = Math.cos(wristAngle);

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
