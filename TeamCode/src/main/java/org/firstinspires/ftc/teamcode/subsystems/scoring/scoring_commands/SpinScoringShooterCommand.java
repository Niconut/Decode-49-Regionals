package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Shooter;

public class SpinScoringShooterCommand extends CommandBase {
    private final Scoring_Shooter scoringShooterSubsystem;
    private final Scoring_Shooter.ScoringShooterState targetState;

    public SpinScoringShooterCommand(Scoring_Shooter subsystem, Scoring_Shooter.ScoringShooterState inputState){
        this.scoringShooterSubsystem = subsystem;
        this.targetState = inputState;
        addRequirements(scoringShooterSubsystem);
    }

    @Override
        public void initialize(){
            scoringShooterSubsystem.setState(targetState);
        }

        public void execute(){
            //scoringShooterSubsystem.shooterTelemetry();
        }

        @Override
        public boolean isFinished(){
            return true;
        }

        @Override
        public void end(boolean interrupted){

        }
}
