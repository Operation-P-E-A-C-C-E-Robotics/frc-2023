package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Util;
import frc.robot.Constants;
import frc.robot.Kinematics;
import frc.robot.Kinematics.SupersystemState;
import frc.robot.Robot;
import frc.robot.RobotState;

//TODO change to use RobotState
public class Supersystem extends SubsystemBase {
    private final Arm arm;
    private final Pivot pivot;
    private final Turret turret;
    private final Wrist wrist;
    private final Kinematics kinematics;

    /**
     * unify motion of entire supersystem
     * @param arm arm subsystem
     * @param pivot pivot subsystem
     * @param turret turret subsystem
     * @param wrist wrist subsystem
     */
    public Supersystem(Arm arm, Pivot pivot, Turret turret, Wrist wrist){
        this.arm = arm;
        this.pivot = pivot;
        this.turret = turret;
        this.wrist = wrist;
        kinematics = new Kinematics(this);
    }

    /**
     * @return the current position of all the supersystem joints
     */
    public SupersystemState getSupersystemState(){
        return new SupersystemState(
            turret.getAngle().getRadians(),
            pivot.getRotation().getRadians(),
            arm.getExtension(),
            wrist.getAngle().getRadians()
        );
    }

    /**
     * @return supersystem kinematics
     */
    public Kinematics getKinematics(){
        return kinematics;
    }

    /**
     * set the position of all supersystem joints
     */
    public void setSupersystemState(SupersystemState state){
        state = Kinematics.optimize(state, getSupersystemState());
        // arm.setExtension(state.getArmExtension());
        turret.setAngle(Rotation2d.fromRadians(state.getTurretAngle()));
        pivot.setAngle(Rotation2d.fromRadians(state.getPivotAngle()));
        // wrist.setAngle(Rotation2d.fromRadians(state.getWristAngle()));
    }

    public boolean withinTolerance(Constants.SupersystemTolerance tolerance){
        return arm.withinTolerance(tolerance)
            && pivot.withinTolerance(tolerance)
            && turret.withinTolerance(tolerance)
            && wrist.withinTolerance(tolerance);
    }

    /**
     * set the position of the end of the lift
     * @param position position from the center of the robot
     */
    public void setSupersystemPosition(Translation3d position){
        //TODO set the wrist angle to be parralel to the ground on the right side, when no other angle is given.
        setSupersystemState(Kinematics.inverseKinematics(position, wrist.getAngle().getRadians()));
    }

    /**
     * set the position of the end of the lift, and the wrist angle
     * @param position position from the center of the robot
     * @param wristAngle the wrist angle - 0 is straight up
     */
    public void setSupersystemPosition(Translation3d position, Rotation2d wristAngle){
        setSupersystemState(Kinematics.inverseKinematics(position, wristAngle.getRadians()));
    }

    /**
     * set the position of the end effector, and the wrist angle
     * @param position position from the center of the robot
     * @param wristAngle the wrist angle - 0 is straight up
     */
    public void setEndEffectorPosition(Translation3d position, Rotation2d wristAngle){
        setSupersystemState(Kinematics.inverseKinematicsFromEndEffector(position, wristAngle.getRadians()));
    }

    /**
     * set the position to the center of where the end effector is held.
     * @param position position from the center of the robot
     * @param wristAngle the wrist angle - 0 is straight up
     */
    public void setPlacePosition(Translation3d position, Rotation2d wristAngle){
        setSupersystemState(Kinematics.inverseKinematicsFromPlacePoint(position, wristAngle.getRadians()));
    }

    /**
     * set the place point to a field relative point
     * @param position the point on the field
     * @param robotState the robot state
     */
    public void setPlacePositionFieldRelative(Translation3d position, RobotState robotState){
        var drivetrainRelativePosition = robotState.fieldToDrivetrain(new Pose3d(position, new Rotation3d()));
        setPlacePosition(drivetrainRelativePosition.getTranslation(), new Rotation2d());
        setWristParallelToGround();
    }

    /**
     * set only the x position of the supersystem
     */
    public Supersystem setX(double x){
        var position = getKinematics().getSupersystemPosition(); //TODO these all need to get the wrist position
        setPlacePosition(
                new Translation3d(x, position.getY(), position.getZ()),
                new Rotation2d(getSupersystemState().getWristAngle())
        );
        return this;
    }

    /**
     * set only the y position of the supersystem
     */
    public Supersystem setY(double y){
        var position = getKinematics().getSupersystemPosition();
        setPlacePosition(
                new Translation3d(position.getX(),y, position.getZ()),
                new Rotation2d(getSupersystemState().getWristAngle())
        );
        return this;
    }

    /**
     * set only the height of the supersystem
     */
    public Supersystem setZ(double z){
        var position = getKinematics().getSupersystemPosition();
        setPlacePosition(
                new Translation3d(position.getX(), position.getY(), z),
                new Rotation2d(getSupersystemState().getWristAngle())
        );
        return this;
    }

    /**
     * set only the turret
     */
    public Supersystem setTurret(Rotation2d position){
        var state = getSupersystemState();
        setSupersystemState(new SupersystemState(
                position.getRadians(),
                state.getPivotAngle(),
                state.getArmExtension(),
                state.getWristAngle()
        ));
        return this;
    }

    /**
     * set only the pivot
     */
    public Supersystem setPivot(Rotation2d position){
        var state = getSupersystemState();
        setSupersystemState(new SupersystemState(
                state.getTurretAngle(),
                position.getRadians(),
                state.getArmExtension(),
                state.getWristAngle()
        ));
        return this;
    }

    /**
     * set only the lift
     */
    public Supersystem setLift(double extension){
        var state = getSupersystemState();
        setSupersystemState(new SupersystemState(
                state.getTurretAngle(),
                state.getPivotAngle(),
                extension,
                state.getWristAngle()
        ));
        return this;
    }

    /**
     * set only the wrist angle
     */
    public Supersystem setWrist(Rotation2d position){
        var state = getSupersystemState();
        setSupersystemState(new SupersystemState(
                state.getTurretAngle(),
                state.getPivotAngle(),
                state.getArmExtension(),
                position.getRadians()
        ));
        return this;
    }

    /**
     * set the wrist angle to be parallel to the ground on the correct side, when no other angle is given.
     * @return this
     */
    public Supersystem setWristParallelToGround(){
        var pivot = getSupersystemState().getPivotAngle();
        var newWrist = (Math.PI/2  + pivot) * Math.signum(pivot);
        if(Util.epsilonEquals(pivot, 0, 0.1)) newWrist = 0;
        System.out.println("PIVOT: " + pivot + " WRIST: " + newWrist);
        setWrist(new Rotation2d(newWrist));
        return this;
    }

    // public Supersystem setDefaultWrist(){
    //     var pivot = getSupersystemState().getPivotAngle();
    //     var 
    // }

    public Supersystem addTurretOffset(double offset){
        setTurret(new Rotation2d(getSupersystemState().getTurretAngle() + offset));
        return this;
    }

    public Supersystem addArmOffset(double offset){
        setLift(getSupersystemState().getArmExtension() + offset);
        return this;
    }

    /**
     * modify the joint angles by another state
     * @param delta the difference between current and target joint angles
     */
    public Supersystem changeStateBy(SupersystemState delta){
        setSupersystemState(getSupersystemState().plus(delta));
        return this;
    }

    /**
     * modify the position of the end of the arm by a translation
     * @param delta difference between current and target positions
     */
    public Supersystem changePositionBy(Translation3d delta){
        setSupersystemPosition(getKinematics().getSupersystemPosition().plus(delta));
        return this;
    }

    public static class SupersystemTolerance{
        private final double armTolerance;
        private final Rotation2d turretTolerance;
        private final Rotation2d pivotTolerance;
        private final Rotation2d wristTolerance;

        public SupersystemTolerance(double armTolerance, Rotation2d turretTolerance, Rotation2d pivotTolerance, Rotation2d wristTolerance){
            this.armTolerance = armTolerance;
            this.turretTolerance = turretTolerance;
            this.pivotTolerance = pivotTolerance;
            this.wristTolerance = wristTolerance;
        }

        public double getArmTolerance() {
            return armTolerance;
        }
        public Rotation2d getTurretTolerance() {
            return turretTolerance;
        }
        public Rotation2d getWristTolerance() {
            return wristTolerance;
        }
        public Rotation2d getPivotTolerance() {
            return pivotTolerance;
        }
    }

    public Subsystem[] getRequirements(){
        return new Subsystem[]{this, turret, pivot, wrist, arm};
    }
    //SIMULATION TESTING:
    Translation3d previousTestSetpoint = new Translation3d();
    @Override
    public void simulationPeriodic(){

    }
}
