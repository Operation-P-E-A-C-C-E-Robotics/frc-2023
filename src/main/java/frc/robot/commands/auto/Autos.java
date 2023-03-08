package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.auto.paths.Paths;
import frc.robot.commands.supersystem.Setpoints;
import frc.robot.subsystems.Supersystem;

public class Autos {
    public static Command testAuto1(Paths paths, RobotState robotState, Supersystem supersystem){
        return paths.testPath(robotState);
//                .andThen(Setpoints.goToSetpoint(
//                        Setpoints.placeHighCube,
//                        supersystem,
//                        Constants.SupersystemTolerance.DEFAULT
//                ).withTimeout(1))
//                        .andThen(
//                                paths.testPath(robotState)
//                        );

    }
}
