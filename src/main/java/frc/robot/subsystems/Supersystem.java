package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Kinematics;
import frc.robot.Kinematics.LiftPosition;
import frc.robot.Kinematics.LiftState;

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

    public LiftState getLiftState(){
        return new LiftState(
            turret.getAngle().getRadians(),
            pivot.getAngle().getDegrees(),
            lift.getExtension(),
            wrist.getAngle().getDegrees()
        );
    }

    public void setLiftState(LiftState state){
        lift.setExtension(state.getLiftExtension());
        turret.setAngle(Rotation2d.fromRadians(state.getTurretAngle()));
        pivot.setAngle(Rotation2d.fromRadians(state.getPivotAngle()));
    }

    public void setLiftPosition(LiftPosition position){
    }
}
