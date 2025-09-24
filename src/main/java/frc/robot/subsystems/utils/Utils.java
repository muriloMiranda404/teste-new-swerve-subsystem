package frc.robot.subsystems.utils;

public class Utils {
    
    public static boolean InRange(double value, double max, double min){
        return value < max && value > min;
    }

    public boolean inReference(double value, double Reference){
        return value == Reference;
    }
}
