package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gate;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Shooter;

public class MoveScoringGateCommand extends CommandBase {
    private final Scoring_Gate scoringGateSubsystem;
    private final Scoring_Gate.ScoringGState targetState;

    public MoveScoringGateCommand(Scoring_Gate subsystem, Scoring_Gate.ScoringGState inputState){
        this.scoringGateSubsystem = subsystem;
        this.targetState = inputState;
        addRequirements(scoringGateSubsystem);
    }

    @Override
        public void initialize(){
            scoringGateSubsystem.setState(targetState);
        }

        public void execute(){
            //nothing to do in the loop
        }

        @Override
        public boolean isFinished(){
            return  true;
        }

        @Override
        public void end(boolean interrupted){

        }
}
