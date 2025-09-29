package frc.robot.subsystems.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controllers;

public class MechanismJoystick implements MechanismJoystickIO{
    private CommandXboxController controller;

    public static MechanismJoystick mInstance = null;

    private MechanismJoystick(){
        this.controller = new CommandXboxController(Controllers.INTAKE_CONTROLLER);
    }
    public static MechanismJoystick getInstance(){
        if(mInstance == null){
            mInstance = new MechanismJoystick();
        }
        return mInstance;
    }

    @Override
    public Trigger L1Button() {
       return controller.a();
    }

    @Override
    public Trigger L2Button() {
        return controller.b();
    }

    @Override
    public Trigger L3Button() {
       return controller.y();
    }

    @Override
    public Trigger L4Button() {
        return controller.x();
    }

    @Override
    public Trigger Processador() {
       return controller.rightBumper().and(controller.leftBumper());
    }

    @Override
    public Trigger Algae_L2() {
        return controller.rightBumper();
    }

    @Override
    public Trigger Algae_L3() {
        return controller.leftBumper();
    }

    @Override
    public double throwCoral(){
        return controller.getRightTriggerAxis() - controller.getLeftTriggerAxis();
    }
}
