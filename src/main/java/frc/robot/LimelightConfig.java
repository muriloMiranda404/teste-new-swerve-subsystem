package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightConfig {
    
    NetworkTable limelight;

    public LimelightConfig(String table){
        limelight = NetworkTableInstance.getDefault().getTable(table);
    }

    public double[] getRobotSpace(){
        return limelight.getEntry("botPose").getDoubleArray(new double[6]);
    }

    public Pose2d getGlobalPose(){
        double[] positions = getRobotSpace();

        if(positions.length <= 6){
            return new Pose2d(
                positions[0],
                positions[1],
                Rotation2d.fromDegrees(positions[5])
            );
        }
        return new Pose2d();
    }

    public double[] getBotPoseBlue(){
        return limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    public boolean getHasTarget(){
        return limelight.getEntry("tv").getDouble(0)==1;
    }
}
