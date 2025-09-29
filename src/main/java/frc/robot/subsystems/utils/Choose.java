package frc.robot.subsystems.chooses;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Choose {
    
    SendableChooser<String> chooser;

    public static Choose mInstance = null;

    private Choose(){
        this.chooser = new SendableChooser<>();

        this.chooser.setDefaultOption("controllers", "");
        this.chooser.addOption("arduino", "arduino");
        this.chooser.addOption("joystick", "joystick");
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
