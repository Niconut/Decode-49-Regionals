package org.firstinspires.ftc.teamcode.subsystems.endgame.Endgame_Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.endgame.Endgame_Kickstand;

public class MoveEndgameKickstandCommand extends CommandBase {
    private final Endgame_Kickstand Endgame_Kickstand;
    private final Endgame_Kickstand.EndgameState targetState;

    public MoveEndgameKickstandCommand(Endgame_Kickstand subsystem, Endgame_Kickstand.EndgameState inputState){
        this.Endgame_Kickstand = subsystem;
        this.targetState = inputState;
        addRequirements(Endgame_Kickstand);
    }

    @Override
    public void initialize(){
        Endgame_Kickstand.setState(targetState);
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