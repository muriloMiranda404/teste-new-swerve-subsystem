package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutonomousCommands {

    public AutonomousCommands(){

    }

    public void configureTest(){
        NamedCommands.registerCommand("PRINT", new InstantCommand(() ->{
            System.out.println("print de autonomous");
        }));
    }
}
