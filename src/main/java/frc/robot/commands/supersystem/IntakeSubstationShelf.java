// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.field.RedField;
import frc.robot.Odometry;
import frc.robot.Kinematics.Position;
import frc.robot.subsystems.Supersystem;

/** TODO NOT AT ALL COMPLETE. */
public class IntakeSubstationShelf extends CommandBase {
    private Supersystem supersystem;

    public static final int INTAKE_CONE_BUTTON = 0, INTAKE_CUBE_BUTTON = 0;
    private Stage stage;

    private Odometry odometry;
    private Translation2d target;

    public IntakeSubstationShelf(Supersystem supersystem, Odometry odometry, Joystick operatorJoystick) {
        this.supersystem = supersystem;
        this.odometry = odometry;
        stage = Stage.TOO_FAR_AWAY;
        addRequirements(supersystem);
    }

    @Override
    public void initialize() {
        target = RedField.nearestLocation(RedField.SUBSTATION_SHELVES, odometry.getPose().getTranslation());
        stage = Stage.TOO_FAR_AWAY;
    }

    @Override
    public void execute() {
        Pose2d pose = odometry.getPose();
        switch(stage){
            case TOO_FAR_AWAY:
                target = RedField.nearestLocation(RedField.SUBSTATION_SHELVES, pose.getTranslation());
                if(pose.getTranslation().getDistance(target) < 0.3) stage = Stage.INTAKE_POSITION_IN_FRONT;
                break;
            case INTAKE_POSITION_IN_FRONT:
                intakeToFieldRelativePoint(target.plus(new Translation2d(0,0)), INTAKE_CONE_BUTTON); //todo offset
                if(supersystem.finishedMotion()) stage = Stage.INTAKE_IN;
                break;
            case INTAKE_IN:
                placePositionToFieldRelativePoint(target, INTAKE_CONE_BUTTON);
                if(supersystem.finishedMotion()) stage = Stage.INTAKE_GRAB;
            case INTAKE_GRAB:

                break;
            case INTAKE_PULL_BACK_AND_UP:
                break;
            case INTAKE_RETRACT:
                break;
            default:
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }

    private void intakeToFieldRelativePoint(Translation2d point, double height){
        Translation2d difference = point.minus(odometry.getPose().getTranslation());
        supersystem.setWristEndPosition(new Position(difference.getX(), difference.getY(), height), Rotation2d.fromDegrees(90));
    }

    private void placePositionToFieldRelativePoint(Translation2d point, double height){
        Translation2d difference = point.minus(odometry.getPose().getTranslation());
        supersystem.setWristPlacePosition(new Position(difference.getX(), difference.getY(), height), Rotation2d.fromDegrees(90));
    }

    private enum Stage{
        TOO_FAR_AWAY,
        INTAKE_POSITION_IN_FRONT,
        INTAKE_IN,
        INTAKE_GRAB,
        INTAKE_PULL_BACK_AND_UP,
        INTAKE_RETRACT
    }
}
