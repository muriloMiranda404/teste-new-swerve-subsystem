package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.autonomousCommands.UpToL2;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.StatesToScore;

public class AutonomousCommands{

  SuperStructure superStructure;

  public AutonomousCommands(){
    this.superStructure = SuperStructure.getInstance();
  }

  public void configureAllCommands(){
    this.configureCoral(this.superStructure);
  }

  private void configureCoral(SuperStructure superStructure){
    NamedCommands.registerCommand("L2", superStructure.ScoreRobot(StatesToScore.L2));
  }
}