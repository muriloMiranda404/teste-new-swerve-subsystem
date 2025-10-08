package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.Constants.Intake.IntakePositions;
import frc.robot.commands.IntakeSpeedCommand;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.SuperStructure.StatesToScore;
import frc.robot.subsystems.mechanism.elevator.ElevatorSubsystem;
import frc.robot.subsystems.mechanism.intake.IntakeSubsystem;

 public class AutonomousCommands{

   private SuperStructure superStructure;
   private IntakeSubsystem intakeSubsystem;
   private ElevatorSubsystem elevatorSubsystem;

   public AutonomousCommands(){
    this.superStructure = SuperStructure.getInstance();
    this.intakeSubsystem = IntakeSubsystem.getInstance();   
    this.elevatorSubsystem = ElevatorSubsystem.getInstance();
  }

   public void configureAllCommands(){
     configureCoral(this.superStructure, this.intakeSubsystem, this.elevatorSubsystem);
   }

   private void configureCoral(SuperStructure superStructure, IntakeSubsystem intakeSubsystem, ElevatorSubsystem elevatorSubsystem){
     NamedCommands.registerCommand("SOLTAR", new IntakeSpeedCommand(0.8).onlyWhile(() -> intakeSubsystem.isTouched()));
     NamedCommands.registerCommand("PEGAR", new IntakeSpeedCommand(true).onlyWhile(() -> !intakeSubsystem.isTouched()));

     NamedCommands.registerCommand("L1", superStructure.ScoreRobot(StatesToScore.L1).until(() -> intakeSubsystem.getSetpoint() == IntakePositions.DEFAULT_POSITION));
     NamedCommands.registerCommand("L2", superStructure.ScoreRobot(StatesToScore.L2).onlyWhile(() -> !intakeSubsystem.atSetpoint()));
     NamedCommands.registerCommand("L3", superStructure.ScoreRobot(StatesToScore.L3).onlyWhile(() -> !intakeSubsystem.atSetpoint()));
     NamedCommands.registerCommand("L4", superStructure.ScoreRobot(StatesToScore.L4).onlyWhile(() -> !intakeSubsystem.atSetpoint()));
    }
  }