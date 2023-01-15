// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.supersystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.JoystickPositionControl;
import frc.robot.Kinematics.Position;
import frc.robot.subsystems.Supersystem;

/** An example command that uses an example subsystem. */
public class KinematicJoystick extends CommandBase {
    private Supersystem supersystem;
    private Joystick joystick;
    private JoystickPositionControl xPositionHelper, yPositionHelper, zPositionHelper;

    public KinematicJoystick(Supersystem supersystem, Joystick operatorJoystick) {
        this.supersystem = supersystem;
        this.joystick = operatorJoystick;
        xPositionHelper = new JoystickPositionControl(0.1, 0.1);
        yPositionHelper = new JoystickPositionControl(0.1, 0.1);
        zPositionHelper = new JoystickPositionControl(0.1, 0.1);
        addRequirements(supersystem);
    }

    @Override
    public void initialize() {
        Position currentPosition = supersystem.getKinematics().getSupersystemPosition();
        xPositionHelper.reset(currentPosition.getX());
        yPositionHelper.reset(currentPosition.getY());
        zPositionHelper.reset(currentPosition.getZ());
    }

    @Override
    public void execute() {
        Rotation2d wrist;
        double x = xPositionHelper.get(joystick.getRawAxis(0));
        double y = xPositionHelper.get(joystick.getRawAxis(1));
        double z = xPositionHelper.get(joystick.getRawAxis(2));
        if (x < 0)  wrist = Rotation2d.fromDegrees(-90);
        else wrist = Rotation2d.fromDegrees(90);
        supersystem.setSupersystemPosition(new Position(x, y, z), wrist);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
