package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.Outros;

public class LimelightConfig {
    
    NetworkTable limelight;

    public static LimelightConfig mInstance = null;

    private LimelightConfig(String table){
        limelight = NetworkTableInstance.getDefault().getTable(table);
    }

    public static LimelightConfig getInstance(){
        if(mInstance == null){
            mInstance = new LimelightConfig(Outros.LIMELIGHT);
        }
        return mInstance;
    }

    public boolean getHasTarget(){
        return limelight.getEntry("tv").getDouble(0 )== 1;
    }

    public double getTagId(){
        return limelight.getEntry("tid").getDouble(-1);
    }

    public double getTx(){
        return limelight.getEntry("tx").getDouble(0.0);
    }

    public double getTy(){
        return limelight.getEntry("ty").getDouble(0.0);
    }

    public double getTa(){
        return limelight.getEntry("ta").getDouble(0.0);
    }

    public boolean setLedMode(int mode){
        return limelight.getEntry("ledMode").setNumber(mode);
    }

    public double[] getRobotSpace(){
        return limelight.getEntry("botPose").getDoubleArray(new double[6]);
    }

    public double[] getBotPoseBlue(){
        return limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
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
    
    public double[] getAprilTagCordenates(){
        return limelight.getEntry("camerapose_robotspace").getDoubleArray(new double[6]);
    }
    
    public double[] getCameraTarget(){
        return limelight.getEntry("camerapose_robotspace_set").getDoubleArray(new double[6]);
    }
}