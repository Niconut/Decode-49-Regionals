package org.firstinspires.ftc.teamcode.subsystems.intake.intake_commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Distance_Sensor;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Subsystem;

public class SpinIntakeSubsystemCommand extends CommandBase {
    private final Intake_Subsystem intakeSubsystem;
    private DistanceUnit distanceUnit;
    public double DISTANCE_MAX = 3.000;
    private final Intake_Subsystem.IntakeSubsystemState targetState;

    public SpinIntakeSubsystemCommand(Intake_Subsystem subsystem, Intake_Subsystem.IntakeSubsystemState inputState){
        this.intakeSubsystem = subsystem;
        this. targetState = inputState;
        addRequirements(intakeSubsystem);
    }

    @Override
        public void initialize(){
            intakeSubsystem.setState(targetState);
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
