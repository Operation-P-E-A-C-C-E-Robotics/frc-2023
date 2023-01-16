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
        Translation2d difference = position.minus(robotPose.getTranslation());
        difference = difference.rotateBy(robotPose.getRotation().unaryMinus()); //TODO positive or negative rotation?
        setWristPlacePosition(new Translation3d(difference.getX(), position.getY(), height), wristAngle);
    }

    public void setX(double x){
        Translation3d position = getKinematics().getSupersystemPosition();
        setWristPlacePosition(new Translation3d(x, position.getY(), position.getZ()), new Rotation2d(getSupersystemState().getWristAngle()));
    }
}
