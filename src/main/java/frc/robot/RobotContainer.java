package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.Controllers;
import frc.robot.Constants.Outros;
import frc.robot.commands.IntakeSpeedCommand;
import frc.robot.commands.ResetPigeon;
import frc.robot.commands.TurnRobot;
import frc.robot.subsystems.joysticks.DriverController;
import frc.robot.subsystems.joysticks.KeyboardMechanism;
import frc.robot.subsystems.joysticks.MechanismJoystick;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.SuperStructure.StatesToScore;
import frc.robot.subsystems.mechanism.intake.IntakeSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.utils.Choose;

public class RobotContainer {
    
  private DriverController driverController;
  private KeyboardMechanism keyBoardControl;
  private MechanismJoystick mechanismJoystick;
    
  private SwerveSubsystem swerveSubsystem;
   
  private IntakeSubsystem intakeSubsystem;
  
  private SuperStructure superStructure;

  private Choose choose;

  public RobotContainer() {
        
    this.driverController = DriverController.getInstance();
    this.keyBoardControl = KeyboardMechanism.getInstance();
    this.mechanismJoystick = MechanismJoystick.getInstance();
        
    this.swerveSubsystem = SwerveSubsystem.getInstance();
    
    this.intakeSubsystem = IntakeSubsystem.getInstance();
    this.superStructure = SuperStructure.getInstance();
    
    this.choose = Choose.getInstance();

    intakeSubsystem.setDefaultCommand(intakeSubsystem.setJoystickControl(mechanismJoystick.throwCoral()));
    
    swerveSubsystem.setDefaultCommand(swerveSubsystem.driveRobot(
      () -> MathUtil.applyDeadband(driverController.ConfigureInputs(1), Controllers.DEADBAND), 
      () -> MathUtil.applyDeadband(driverController.ConfigureInputs(2), Controllers.DEADBAND), 
      () -> MathUtil.applyDeadband(driverController.ConfigureInputs(3), Controllers.DEADBAND),
      true));
      
      configureJoystickMechanismBindings();
      
      configureKeyBoardMechanismBiding();
      configureDriveBindings();
  }

  private void configureJoystickMechanismBindings(){
    mechanismJoystick.L1Button().onTrue(new ParallelCommandGroup(
      superStructure.ScoreRobot(StatesToScore.L1),
      new IntakeSpeedCommand(true)
    ));
    mechanismJoystick.L2Button().onTrue(superStructure.ScoreRobot(StatesToScore.L2));
    mechanismJoystick.L3Button().onTrue(superStructure.ScoreRobot(StatesToScore.L3));
    mechanismJoystick.L4Button().onTrue(superStructure.ScoreRobot(StatesToScore.L4));

    mechanismJoystick.Algae_L2().onTrue(superStructure.ScoreRobot(StatesToScore.ALGAE_L2));
    mechanismJoystick.Algae_L3().onTrue(superStructure.ScoreRobot(StatesToScore.ALGAE_L3));
    mechanismJoystick.Processador().onTrue(superStructure.ScoreRobot(StatesToScore.PROCESSADOR));
  }
  
  private void configureKeyBoardMechanismBiding() {
    keyBoardControl.L1Button().onTrue(new ParallelCommandGroup(
      new IntakeSpeedCommand(true),
      superStructure.ScoreRobot(StatesToScore.L1)
    ));
    keyBoardControl.L2Button().onTrue(superStructure.ScoreRobot(StatesToScore.L2));
    keyBoardControl.L3Button().onTrue(superStructure.ScoreRobot(StatesToScore.L3));
    keyBoardControl.L4Button().onTrue(superStructure.ScoreRobot(StatesToScore.L4));
    
    keyBoardControl.throwCoral().whileTrue(new IntakeSpeedCommand(0.8));
    keyBoardControl.AlgaeL2().onTrue(superStructure.ScoreRobot(StatesToScore.ALGAE_L2));
    keyBoardControl.AlgaeL3().onTrue(superStructure.ScoreRobot(StatesToScore.ALGAE_L3));
    keyBoardControl.Processador().onTrue(superStructure.ScoreRobot(StatesToScore.PROCESSADOR));
  }
  
  private void configureDriveBindings(){
    driverController.resetPigeon().onTrue(new ResetPigeon());

    driverController.turn45().onTrue(new TurnRobot( 45));
    driverController.turn315().onTrue(new TurnRobot(-45));    
  }
  
 public Command getAutonomousCommand() {
    return swerveSubsystem.getAutonomousCommand(Outros.AUTO, true);
  }

  public void setMotorBrake(boolean brake){
    swerveSubsystem.setMotorBrake(brake);
  }
}
