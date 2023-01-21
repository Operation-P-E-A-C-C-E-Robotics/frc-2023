package frc.lib.sensors;

import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ApriltagLimelight {
    NetworkTableInstance networkTables = NetworkTableInstance.getDefault();
    NetworkTable limelight = networkTables.getTable("limelight");
    DoubleArrayTopic botposeTopic = limelight.getDoubleArrayTopic("botpose");

    
}
