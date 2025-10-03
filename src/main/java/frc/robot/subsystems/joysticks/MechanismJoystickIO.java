package frc.robot.subsystems.joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface MechanismJoystickIO {
 
    Trigger L1Button();

    Trigger L2Button();
    
    Trigger L3Button();

    Trigger L4Button();

    Trigger Processador();
    
    Trigger Algae_L2();

    Trigger Algae_L3();

    double throwCoral();

    double getRightTrigger();

    double getLeftTrigger();
}