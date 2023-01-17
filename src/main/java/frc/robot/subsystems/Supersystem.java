package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Kinematics;
import frc.robot.Kinematics.SupersystemState;

public class Supersystem extends SubsystemBase {
    private final Arm arm;
    private final Pivot pivot;
    private final Turret turret;
    private final Wrist wrist;
    private final Kinematics kinematics;

    public Supersystem(Arm arm, Pivot pivot, Turret turret, Wrist wrist){
        this.arm = arm;
        this.pivot = pivot;
        this.turret = turret;
        this.wrist = wrist;
        kinematics = new Kinematics(this);
    }

    public SupersystemState getSupersystemState(){
        return new SupersystemState(
            turret.getAngle().getRadians(),
            pivot.getAngle().getRadians(),
            arm.getExtension(),
            wrist.getAngle().getRadians()
        );
    }

    public Kinematics getKinematics(){
        return kinematics;
    }

    public boolean finishedMotion(){
        return arm.finishedMotion()
            && pivot.finishedMotion()
            && turret.finishedMotion()
            && wrist.finishedMotion();
    }

    public void setSupersystemState(SupersystemState state){
        arm.setExtension(state.getArmExtension());
        turret.setAngle(Rotation2d.fromRadians(state.getTurretAngle()));
        pivot.setAngle(Rotation2d.fromRadians(state.getPivotAngle()));
    }

    public void setSupersystemPosition(Translation3d position){
        setSupersystemState(Kinematics.inverseKinematics(position, wrist.getAngle().getRadians()));
    }

    public void setSupersystemPosition(Translation3d position, Rotation2d wristAngle){
        setSupersystemState(Kinematics.inverseKinematics(position, wristAngle.getRadians()));
    }

    public void setEndEffectorPosition(Translation3d position, Rotation2d wristAngle){
        setSupersystemState(Kinematics.inverseKinematicsFromEndEffector(position, wristAngle.getRadians()));
    }

    public void setPlacePosition(Translation3d position, Rotation2d wristAngle){
        setSupersystemState(Kinematics.inverseKinematicsFromPlacePoint(position, wristAngle.getRadians()));
    }

    public void setPlacePositionFieldRelative(Translation2d position, Pose2d robotPose, double height, Rotation2d wristAngle){
        var difference = position.minus(robotPose.getTranslation());
        difference = difference.rotateBy(robotPose.getRotation().unaryMinus()); //TODO positive or negative rotation?
        setPlacePosition(new Translation3d(difference.getX(), position.getY(), height), wristAngle);
    }

    public Supersystem setX(double x){
        var position = getKinematics().getSupersystemPosition(); //TODO these all need to get the wrist position
        setPlacePosition(
                new Translation3d(x, position.getY(), position.getZ()),
                new Rotation2d(getSupersystemState().getWristAngle())
        );
        return this;
    }

    public Supersystem setY(double y){
        var position = getKinematics().getSupersystemPosition();
        setPlacePosition(
                new Translation3d(position.getX(),y, position.getZ()),
                new Rotation2d(getSupersystemState().getWristAngle())
        );
        return this;
    }

    public Supersystem setZ(double z){
        var position = getKinematics().getSupersystemPosition();
        setPlacePosition(
                new Translation3d(position.getX(), position.getY(), z),
                new Rotation2d(getSupersystemState().getWristAngle())
        );
        return this;
    }

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

    public Supersystem changeStateBy(SupersystemState delta){
        setSupersystemState(getSupersystemState().plus(delta));
        return this;
    }

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
}
