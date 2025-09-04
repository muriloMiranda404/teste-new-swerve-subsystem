// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.swerve;
import frc.robot.subsystems.SwerveSubsystem;


public class RobotContainer {

  SwerveSubsystem subsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  XboxController joystick = new XboxController(0);
  AutonomousCommands autonomousCommands= new AutonomousCommands();

  public RobotContainer() {

    subsystem.setDefaultCommand(subsystem.driveCommand(
      () -> MathUtil.applyDeadband(joystick.getLeftY(), 0),
      () -> MathUtil.applyDeadband(joystick.getLeftX(), 0),
      () -> MathUtil.applyDeadband(joystick.getRightX(), 0)
      ));
    autonomousCommands.configureTest();
    configureBindings();
  }

  private void configureBindings() {

  }

 public Command getAutonomousCommand() {
    try{
        PathPlannerPath path = PathPlannerPath.fromPathFile(swerve.AUTO);

        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }

  public void setMotorBrake(boolean brake){
    subsystem.setMotorBrake(brake);
  }
}
