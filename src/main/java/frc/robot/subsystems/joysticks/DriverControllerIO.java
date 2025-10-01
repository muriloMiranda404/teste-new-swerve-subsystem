package frc.robot.subsystems.joysticks;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface DriverControllerIO {
    
    boolean activateMarcha();

    boolean isInvertedAlliance();

    double getLeftX();

    double getLeftY();

    double getRightX();

    double getRightY();

    double ConfigureInputs( int choose);

    Trigger alingToReefButton();

    Trigger turn45();

    Trigger turn315();
}
