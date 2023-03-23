package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.DriveSignal;
import frc.robot.RobotState;
import frc.robot.subsystems.DriveTrain;

public class MobilityOverStation extends CommandBase {
    Stage stage = Stage.DRIVING_TO;
    private DriveTrain driveTrain;
    private RobotState robotState;

    public MobilityOverStation(DriveTrain driveTrain, RobotState robotState){
        this.driveTrain = driveTrain;
        this.robotState = robotState;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize(){
        stage = Stage.DRIVING_TO;
    }

    @Override
    public void execute(){
        DriveSignal signal = DriveSignal.DEFAULT;
        double pitch = robotState.getPigeon().getRoll();
        switch(stage){
            case DRIVING_TO:
                signal = DriveSignal.tankDrive(0.4, 0.4, true);
                if(Math.abs(pitch) > 10) stage = Stage.DRIVING_ON;
                break;
            case DRIVING_ON:
                signal = DriveSignal.tankDrive(0.3, 0.3, true);
                if(Math.abs(pitch) < 5) stage = Stage.DRIVING_OVER;
            case DRIVING_OVER:
                signal = DriveSignal.tankDrive(0.1, 0.1, true);
                if(Math.abs(pitch) > 10) stage = Stage.DRIVING_OFF;
                break;
            case DRIVING_OFF:
                signal = DriveSignal.tankDrive(0.2, 0.2, true);
                if(Math.abs(pitch) < 5) stage = Stage.DONE;
                break;
            case DONE:
                break;
        }
        driveTrain.set(signal);
    }

    @Override
    public boolean isFinished(){
        return stage == Stage.DONE;
    }

    enum Stage{
        DRIVING_TO,
        DRIVING_ON,
        DRIVING_OVER,
        DRIVING_OFF,
        DONE
    }
}
