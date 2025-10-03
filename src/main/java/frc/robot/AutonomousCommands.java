package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.SuperStructure.StatesToScore;

 public class AutonomousCommands{

   private SuperStructure superStructure;

   public AutonomousCommands(){
     this.superStructure = SuperStructure.getInstance();
   }

   public void configureAllCommands(){
     configureCoral(this.superStructure);
   }

   private void configureCoral(SuperStructure superStructure){

    NamedCommands.registerCommand("TESTE", new InstantCommand(() -> superStructure.ScoreRobot(StatesToScore.L2)));

    new EventTrigger("TESTE").onTrue(new InstantCommand(() -> superStructure.ScoreRobot(StatesToScore.L2)));
  }
}