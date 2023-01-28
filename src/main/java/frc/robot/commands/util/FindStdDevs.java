package frc.robot.commands.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;

import java.util.ArrayList;

public class FindStdDevs extends CommandBase {
    private final RobotState state;
    private final ArrayList<Double> xMeasurements = new ArrayList<Double>();
    private final ArrayList<Double> yMeasurements = new ArrayList<Double>();
    private final ArrayList<Double> zMeasurements = new ArrayList<Double>();
    private final ArrayList<Double> yawMeasurements = new ArrayList<Double>();
    private final ArrayList<Double> pitchMeasurements = new ArrayList<Double>();
    private final ArrayList<Double> rollMeasurements = new ArrayList<Double>();

    public FindStdDevs(RobotState state){
        this.state = state;
    }

    @Override
    public void initialize(){
        System.out.println("starting standard deviation measurement!");
        System.out.println("don't move the robot.");
        xMeasurements.clear();
        yMeasurements.clear();
        zMeasurements.clear();
        yawMeasurements.clear();
        pitchMeasurements.clear();
        rollMeasurements.clear();
    }

    @Override
    public void execute(){
        var newPose = state.getRawApriltagBotpose();
        if(newPose.getX() == 0) return;
        xMeasurements.add(newPose.getX());
        yMeasurements.add(newPose.getY());
        zMeasurements.add(newPose.getZ());
        rollMeasurements.add(newPose.getRotation().getX());
        pitchMeasurements.add(newPose.getRotation().getY());
        yawMeasurements.add(newPose.getRotation().getZ());

        SmartDashboard.putNumber("X stdev", calcStdDev(xMeasurements));
        SmartDashboard.putNumber("Y stdev", calcStdDev(yMeasurements));
        SmartDashboard.putNumber("Z stdev", calcStdDev(zMeasurements));
        SmartDashboard.putNumber("Roll stdev", calcStdDev(rollMeasurements));
        SmartDashboard.putNumber("Pitch stdev", calcStdDev(pitchMeasurements));
        SmartDashboard.putNumber("Yaw stdev", calcStdDev(yawMeasurements));
    }

    private  double calcStdDev(ArrayList<Double> values){
        double mean = 0;
        for(var i : values){
            mean += i;
        }
        mean /= values.size();

        double stdv = 0;
        for (var i : values){
            stdv += Math.pow(Math.abs(i - mean), 2);
        }
        stdv /= values.size();
        stdv = Math.sqrt(stdv);
        return stdv;
    }
}
