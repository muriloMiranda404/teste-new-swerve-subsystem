package frc.robot.subsystems.joysticks;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class KeyboardMechanism implements KeyboardIO{
    
    private CommandXboxController controller;

    public static KeyboardMechanism mInstance = null;

    public KeyboardMechanism(){
        this.controller = new CommandXboxController(2
        
        
        
        );
    }

    public static KeyboardMechanism getInstance(){
        if(mInstance == null){
            mInstance = new KeyboardMechanism();
        } 
        return mInstance;
    }

    @Override
    public Trigger L1Button() {
        return controller.button(1);
    }

    @Override
    public Trigger L2Button() {
        return controller.button(2);
    }

    @Override
    public Trigger L3Button() {
        return controller.button(3);
    }

    @Override
    public Trigger L4Button() {
        return controller.button(4);
    }

    @Override
    public Trigger AlgaeL2() {
        return controller.button(8);
    }

    @Override
    public Trigger AlgaeL3() {
        return controller.button(9);
    }

    @Override
    public Trigger Processador() {
        return controller.button(7);
    }

    @Override
    public Trigger throwCoral(){
        return controller.button(5);
    }
}
