package frc.robot.subsystems.utils;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IntakeControllerIO {
 
    Trigger L1Button();

    Trigger L2Button();
    
    Trigger L3Button();

    Trigger L4Button();

    Trigger Processador();
    
    Trigger Algae_L2();

    Trigger Algae_L3();

    Trigger getCoral();

    Trigger throwCoral();

    GenericHID getHID();
}
