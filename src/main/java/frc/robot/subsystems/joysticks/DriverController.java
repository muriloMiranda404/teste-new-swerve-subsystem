package frc.robot.subsystems.joysticks;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.SwerveSubsystem;

public class DriverController implements DriverControllerIO{

    CommandXboxController controller;
    SwerveSubsystem swerve;

    public static DriverController mInstance = null;

    private DriverController(){
        controller = new CommandXboxController(0);
        swerve = SwerveSubsystem.getInstance();
    }

    public static DriverController getInstance(){
        if(mInstance == null){
            mInstance = new DriverController();
        }
        return mInstance;
    }

    public double ConfigureInputs(int choose){

        double marcha;
        double invert;

            if(activateMarcha()){
                marcha =  0.7 + (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());

                if(marcha < 0.45){
                    marcha = 0.45;
                }
            } else{
                marcha = 0.7;
            }

            if(isInvertedAlliance()){
                invert = -1.0;
            } else{
                invert = 1.0;
            }

            switch (choose) {
                case 1:
                    
                    return controller.getLeftY() * marcha * invert;
            
                case 2:
                
                    return controller.getLeftX() * marcha * invert;

                case 3:
                
                    return controller.getRightX() * marcha;
        }
        return choose;
    }

    @Override
    public boolean activateMarcha() {
     return ((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) != 0);
    }

    public Trigger resetPigeon(){
        return controller.button(9);
    }

    @Override
    public boolean isInvertedAlliance() {
        return DriverStation.getAlliance().get() == Alliance.Red;
    }

    @Override
    public double getLeftX() {
        return controller.getLeftX();
    }

    @Override
    public double getLeftY() {
        return controller.getLeftY();
    }

    @Override
    public double getRightX() {
        return controller.getRightX();
    }

    @Override
    public double getRightY() {
       return controller.getRightY();
    }

    @Override
    public Trigger alingToReefButton() {
        return controller.rightBumper();
    }

    public Trigger turn45(){
        return controller.a();
    }

    public Trigger turn315(){
        return controller.b();
    }
}
