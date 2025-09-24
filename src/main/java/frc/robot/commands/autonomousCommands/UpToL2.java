package frc.robot.commands.autonomousCommands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Elevator.ElevatorPositions;
import frc.robot.Constants.Intake.IntakePositions;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.commands.IntakePositionCommand;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.StatesToScore;

public class UpToL2 extends SequentialCommandGroup{
    
    SuperStructure superStructure;

    public UpToL2(SuperStructure structure){
        this.superStructure = structure;
        addCommands(
            new InstantCommand(() -> System.out.println("inicializado")),
            new IntakePositionCommand(IntakePositions.PUT_CORAL).
            andThen(new ElevatorPositionCommand(ElevatorPositions.L2))
        );
    }
}
