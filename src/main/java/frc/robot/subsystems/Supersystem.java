package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Kinematics;
import frc.robot.Kinematics.Position;
import frc.robot.Kinematics.SupersystemState;

public class Supersystem extends SubsystemBase {
    private Lift lift;
    private Pivot pivot;
    private Turret turret;
    private Wrist wrist;
    private Kinematics kinematics;

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

    public void setSupersystemState(SupersystemState state){
        lift.setExtension(state.getLiftExtension());
        turret.setAngle(Rotation2d.fromRadians(state.getTurretAngle()));
        pivot.setAngle(Rotation2d.fromRadians(state.getPivotAngle()));
    }

    public void setSupersystemPosition(Position position){
        setSupersystemState(Kinematics.inverseKinematics(position, wrist.getAngle().getRadians()));
    }

    public void setSupersystemPosition(Position position, Rotation2d wristAngle){
        setSupersystemState(Kinematics.inverseKinematics(position, wristAngle.getRadians()));
    }

    public void setWristEndPosition(Position position, Rotation2d wristAngle){
        setSupersystemState(Kinematics.inverseKinematicsFromWristEnd(position, wristAngle.getRadians()));
    }

    public void setWristPlacePosition(Position position, Rotation2d wristAngle){
        setSupersystemState(Kinematics.inverseKinematicsFromWristPlacePoint(position, wristAngle.getRadians()));
    }
}
