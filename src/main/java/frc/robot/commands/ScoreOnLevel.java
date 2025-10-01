package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mechanism.SuperStructure;
import frc.robot.subsystems.mechanism.SuperStructure.StatesToScore;

public class ScoreOnLevel extends Command{
    
    SuperStructure superStructure;
    StatesToScore state;

    public ScoreOnLevel(StatesToScore state){
        this.superStructure = SuperStructure.getInstance();
        this.state = state;
        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        superStructure.ScoreRobot(state);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
