package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IntakeSpeedCommand;
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
     NamedCommands.registerCommand("SOLTAR", new IntakeSpeedCommand(0.8));
     NamedCommands.registerCommand("PEGAR", new IntakeSpeedCommand(-0.5));
     NamedCommands.registerCommand("L2", superStructure.ScoreRobot(StatesToScore.L2));
     NamedCommands.registerCommand("L3", superStructure.ScoreRobot(StatesToScore.L3));
     NamedCommands.registerCommand("L4", superStructure.ScoreRobot(StatesToScore.L4));
    }
  }