// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.supersystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Supersystem;

/** An example command that uses an example subsystem. */
public class PlaceHigh extends CommandBase {
    private Supersystem supersystem;

    public PlaceHigh(Supersystem supersystem) {
        this.supersystem = supersystem;
        addRequirements(supersystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
