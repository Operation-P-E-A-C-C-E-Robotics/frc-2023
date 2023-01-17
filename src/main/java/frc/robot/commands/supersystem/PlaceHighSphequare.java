// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.field.RedField;
import frc.robot.RobotState;
import frc.robot.subsystems.Supersystem;

/** An example command that uses an example subsystem. */
public class PlaceHighSphequare extends CommandBase {
    private final RobotState robotState;
    private Supersystem supersystem;
    private Translation3d target;

    public PlaceHighSphequare(RobotState robotState, Supersystem supersystem) {
        this.supersystem = supersystem;
        this.robotState = robotState;
        target = null;
        addRequirements(supersystem);
    }

    @Override
    public void initialize() {
        target = RedField.nearestLocation(RedField.TOP_SPHEQUARE_SCORE_LOCATIONS, robotState.fieldToDrivetrain(new Pose3d()).getTranslation());
        //figure out where the end effector needs to be.
        Pose3d robotRelativeTarget = robotState.fieldToDrivetrain(new Pose3d(target, new Rotation3d()));
        supersystem.setEndEffectorPosition(robotRelativeTarget.getTranslation(), Rotation2d.fromDegrees(90)); //TODO get good end effector position
    }

    @Override
    public void execute() {
        if (supersystem.finishedMotion()) {
            //TODO EJECT
        }
        //TODO end when game piece ejected.
    }

    @Override
    public void end(boolean interrupted) {
        //TODO end when game piece ejected.
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
