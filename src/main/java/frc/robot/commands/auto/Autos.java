package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.commands.auto.paths.Paths;
import frc.robot.commands.supersystem.Automations;
import frc.robot.commands.supersystem.Setpoints;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Supersystem;

public class Autos {
    private final Setpoints setpoints;
    private final Automations automations;
    private final EndEffector endEffector;
    private final Supersystem supersystem;
    private final Paths paths;
    private final RobotState robotState;

    public Autos(Setpoints setpoints, Automations automations, EndEffector endEffector, Supersystem supersystem, Paths paths, RobotState robotState){
        this.setpoints = setpoints;
        this.automations = automations;
        this.endEffector = endEffector;
        this.supersystem = supersystem;
        this.paths = paths;
        this.robotState = robotState;
    }

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

    public Command simpleAutoPlace(Automations.PlaceLevel level,  boolean isCone){
        return endEffector.grabCommand(true).andThen(
                setpoints.goToPlaceAndEnd(level, isCone),
                endEffector.grabCommand(false),
                endEffector.ejectCommand().unless(() -> isCone),
                new WaitCommand(0.5),
                setpoints.zero(),
                new WaitCommand(0.5)
        );
    }
}
