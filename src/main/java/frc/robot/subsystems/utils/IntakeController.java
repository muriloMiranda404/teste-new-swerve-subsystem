package frc.robot.subsystems.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Controllers;

public class IntakeController implements IntakeControllerIO{
    private CommandXboxController controller;

    public static IntakeController mInstance = null;

    private IntakeController(){
        this.controller = new CommandXboxController(Controllers.INTAKE_CONTROLLER);
    }
    public static IntakeController getInstance(){
        if(mInstance == null){
            mInstance = new IntakeController();
        }
        return mInstance;
    }

    @Override
    public GenericHID getHID() {
        return controller.getHID();
    }

    @Override
    public Trigger L1Button() {
       return controller.button(1);
    }

    @Override
    public Trigger L2Button() {
        return controller.b();
    }

    @Override
    public Trigger L3Button() {
       return controller.x();
    }

    @Override
    public Trigger L4Button() {
        return controller.y();
    }

    @Override
    public Trigger getCoral() {
        return controller.button(5);
    }

    @Override
    public Trigger throwCoral() {
        return controller.rightBumper();
    }

    @Override
    public Trigger Processador() {
       return controller.button(7);
    }

    @Override
    public Trigger Algae_L2() {
        return controller.button(8);
    }

    @Override
    public Trigger Algae_L3() {
        return controller.button(9);
    }
}
