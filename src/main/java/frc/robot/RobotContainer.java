package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Controllers;
import frc.robot.Constants.Outros;
import frc.robot.commands.IntakeSpeedCommand;
import frc.robot.commands.ResetPigeon;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightConfig;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SuperStructure.StatesToScore;
import frc.robot.subsystems.utils.DriverController;
import frc.robot.subsystems.utils.IntakeController;


public class RobotContainer {

  private DriverController driverController;
  private IntakeController intakeController;

  private LimelightConfig limelightConfig;

  private SwerveSubsystem swerveSubsystem;

  private IntakeSubsystem intakeSubsystem;
  private ElevatorSubsystem elevatorSubsystem;
  private SuperStructure superStructure;

  private AutonomousCommands autonomousCommands;

  public RobotContainer() {

    this.driverController = DriverController.getInstance();
    this.intakeController = IntakeController.getInstance();

    this.limelightConfig = LimelightConfig.getInstance();

    this.swerveSubsystem = SwerveSubsystem.getInstance();

    this.elevatorSubsystem = ElevatorSubsystem.getInstance();
    this.intakeSubsystem = IntakeSubsystem.getInstance();
    this.superStructure = SuperStructure.getInstance();
    
    this.autonomousCommands = new AutonomousCommands();
    
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveRobot(
      () -> MathUtil.applyDeadband(driverController.ConfigureInputs(1), Controllers.DEADBAND), 
      () -> MathUtil.applyDeadband(driverController.ConfigureInputs(2), Controllers.DEADBAND), 
      () -> MathUtil.applyDeadband(driverController.ConfigureInputs(3), Controllers.DEADBAND),
      true));

      configureAuto();
      configureBindings();
  }

  
  private void configureBindings() {
    
      intakeController.L1Button().onTrue(new ParallelCommandGroup(
        new IntakeSpeedCommand(true),
        superStructure.ScoreRobot(StatesToScore.L1)
      ));
      intakeController.L2Button().onTrue(superStructure.ScoreRobot(StatesToScore.L2));
      intakeController.L3Button().onTrue(superStructure.ScoreRobot(StatesToScore.L3));
      intakeController.L4Button().onTrue(superStructure.ScoreRobot(StatesToScore.L4));
      
      intakeController.throwCoral().whileTrue(new IntakeSpeedCommand(0.8));
      intakeController.Algae_L2().onTrue(superStructure.ScoreRobot(StatesToScore.ALGAE_L2));
      intakeController.Algae_L3().onTrue(superStructure.ScoreRobot(StatesToScore.ALGAE_L3));
      intakeController.Processador().onTrue(superStructure.ScoreRobot(StatesToScore.PROCESSADOR));
      
      driverController.resetPigeon().onTrue(new ResetPigeon());
  }
    
  private void configureAuto() {
    this.autonomousCommands.configureAllCommands();
  }
  
 public Command getAutonomousCommand() {
    return swerveSubsystem.getAutonomousCommand(Outros.AUTO, true);
  }

  public void setMotorBrake(boolean brake){
    swerveSubsystem.setMotorBrake(brake);
  }
}
