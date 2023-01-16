package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Kinematics;
import frc.robot.Kinematics.SupersystemState;

public class Supersystem extends SubsystemBase {
    private final Lift lift;
    private final Pivot pivot;
    private final Turret turret;
    private final Wrist wrist;
    private final Kinematics kinematics;

    public Supersystem(Lift lift, Pivot pivot, Turret turret, Wrist wrist){
        this.lift = lift;
        this.pivot = pivot;
        this.turret = turret;
        this.wrist = wrist;
        kinematics = new Kinematics(this);
    }

    public SupersystemState getSupersystemState(){
        return new SupersystemState(
            turret.getAngle().getRadians(),
            pivot.getAngle().getRadians(),
            lift.getExtension(),
            wrist.getAngle().getRadians()
        );
    }

    public Kinematics getKinematics(){
        return kinematics;
    }

    public boolean finishedMotion(){
        return lift.finishedMotion()
            && pivot.finishedMotion()
            && turret.finishedMotion()
            && wrist.finishedMotion();
    }

    public void setSupersystemState(SupersystemState state){
        lift.setExtension(state.getLiftExtension());
        turret.setAngle(Rotation2d.fromRadians(state.getTurretAngle()));
        pivot.setAngle(Rotation2d.fromRadians(state.getPivotAngle()));
    }

    public void setSupersystemPosition(Translation3d position){
        setSupersystemState(Kinematics.inverseKinematics(position, wrist.getAngle().getRadians()));
    }

    public void setSupersystemPosition(Translation3d position, Rotation2d wristAngle){
        setSupersystemState(Kinematics.inverseKinematics(position, wristAngle.getRadians()));
    }

    public void setWristEndPosition(Translation3d position, Rotation2d wristAngle){
        setSupersystemState(Kinematics.inverseKinematicsFromWristEnd(position, wristAngle.getRadians()));
    }

    public void setWristPlacePosition(Translation3d position, Rotation2d wristAngle){
        setSupersystemState(Kinematics.inverseKinematicsFromWristPlacePoint(position, wristAngle.getRadians()));
    }

    public void setWristPlacePositionFieldRelative(Translation2d position, Pose2d robotPose, double height, Rotation2d wristAngle){
        var difference = position.minus(robotPose.getTranslation());
        difference = difference.rotateBy(robotPose.getRotation().unaryMinus()); //TODO positive or negative rotation?
        setWristPlacePosition(new Translation3d(difference.getX(), position.getY(), height), wristAngle);
    }

    public Supersystem setX(double x){
        var position = getKinematics().getSupersystemPosition(); //TODO these all need to get the wrist position
        setWristPlacePosition(
                new Translation3d(x, position.getY(), position.getZ()),
                new Rotation2d(getSupersystemState().getWristAngle())
        );
        return this;
    }

    public Supersystem setY(double y){
        var position = getKinematics().getSupersystemPosition();
        setWristPlacePosition(
                new Translation3d(position.getX(),y, position.getZ()),
                new Rotation2d(getSupersystemState().getWristAngle())
        );
        return this;
    }

    public Supersystem setZ(double z){
        var position = getKinematics().getSupersystemPosition();
        setWristPlacePosition(
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
                state.getLiftExtension(),
                state.getWristAngle()
        ));
        return this;
    }

    public Supersystem setPivot(Rotation2d position){
        var state = getSupersystemState();
        setSupersystemState(new SupersystemState(
                state.getTurretAngle(),
                position.getRadians(),
                state.getLiftExtension(),
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
                state.getLiftExtension(),
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
}
