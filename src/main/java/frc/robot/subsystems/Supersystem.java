package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Kinematics;
import frc.robot.Kinematics.SupersystemState;
import frc.robot.Robot;

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
        if(Robot.isSimulation() && PERIODIC_CONTROL_SIMULATION) {
            SmartDashboard.putNumber("supersystem test x", 0);
            SmartDashboard.putNumber("supersystem test y", 0);
            SmartDashboard.putNumber("supersystem test z", 0);
        }
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

    //TODO change to tolerance based thing
    public boolean finishedMotion(){
        return false;
        // return arm.finishedMotion()
        //     && pivot.finishedMotion()
        //     && turret.finishedMotion()
        //     && wrist.withinTolerance()
    }

    /**
     * set the position of all supersystem joints
     */
    public void setSupersystemState(SupersystemState state){
        state = Kinematics.optimize(state, getSupersystemState());
        arm.setExtension(state.getArmExtension());
        turret.setAngle(Rotation2d.fromRadians(state.getTurretAngle()));
        pivot.setAngle(Rotation2d.fromRadians(state.getPivotAngle()));
        wrist.setAngle(Rotation2d.fromRadians(state.getWristAngle()));
    }

    /**
     * set the position of the end of the lift
     * @param position position from the center of the robot
     */
    public void setSupersystemPosition(Translation3d position){
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
     * @param robotPose the robot's pose
     * @param height the height above the field translation
     * @param wristAngle the angle of the wrist - 0 is straight up.
     */
    public void setPlacePositionFieldRelative(Translation2d position, Pose2d robotPose, double height, Rotation2d wristAngle){
        var difference = position.minus(robotPose.getTranslation());
        difference = difference.rotateBy(robotPose.getRotation().unaryMinus()); //TODO positive or negative rotation?
        setPlacePosition(new Translation3d(difference.getX(), position.getY(), height), wristAngle);
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
    private final boolean PERIODIC_CONTROL_SIMULATION = false;
    @Override
    public void simulationPeriodic(){
        var newSetpoint = new Translation3d(
                SmartDashboard.getNumber("supersystem test x", 0),
                SmartDashboard.getNumber("supersystem test y", 0),
                SmartDashboard.getNumber("supersystem test z", 0)
        );
        if(!newSetpoint.equals(previousTestSetpoint) && PERIODIC_CONTROL_SIMULATION) {
            setPlacePosition(newSetpoint, new Rotation2d());
            previousTestSetpoint = newSetpoint;
        }
    }
}
