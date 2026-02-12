package org.firstinspires.ftc.teamcode.subsystems.intake.intake_commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Indexer;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Subsystem;

public class SpinIndexerCommand extends CommandBase {
    private final Intake_Indexer intakeIndexer;
    private final Intake_Indexer.IntakeIndexerState targetState;

    public SpinIndexerCommand(Intake_Indexer subsystem, Intake_Indexer.IntakeIndexerState inputState){
        this.intakeIndexer = subsystem;
        this.targetState = inputState;
        addRequirements(intakeIndexer);
    }

    @Override
        public void initialize(){
            intakeIndexer.setState(targetState);
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
