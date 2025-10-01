package frc.robot.subsystems.utils;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Choose {
    
    SendableChooser<String> chooser;

    public static Choose mInstance = null;

    private Choose(){
        this.chooser = new SendableChooser<>();
        
        this.chooser.setDefaultOption("controllers", "default");
        this.chooser.addOption("arduino", "arduino");
        this.chooser.addOption("joystick", "joystick");
        
        SmartDashboard.putData("chooser", chooser);
    }

    public static Choose getInstance(){
        if(mInstance == null){
            mInstance = new Choose();
        }
        return mInstance;
    }

    public String getChoosed(){
        return chooser.getSelected();
    }
}
