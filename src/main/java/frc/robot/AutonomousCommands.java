package frc.robot;

 import com.pathplanner.lib.auto.NamedCommands;

 import frc.robot.subsystems.mechanism.SuperStructure;
 import frc.robot.subsystems.mechanism.SuperStructure.StatesToScore;

 public class AutonomousCommands{

   private static SuperStructure superStructure;

   public AutonomousCommands(){
     AutonomousCommands.superStructure = SuperStructure.getInstance();
   }

   public static void configureAllCommands(){
     configureCoral(AutonomousCommands.superStructure);
   }

   private static void configureCoral(SuperStructure superStructure){
     NamedCommands.registerCommand("L2", superStructure.ScoreRobot(StatesToScore.L2));
   }
}