package org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Sensors.Light_Indicator;

public class ActuateLightIndicatorCommand extends CommandBase {
        private final Light_Indicator lightIndicator;
        private final Light_Indicator.LightIndicatorState targetState;

    public ActuateLightIndicatorCommand(Light_Indicator subsystem, Light_Indicator.LightIndicatorState inputState){
        lightIndicator = subsystem;
        targetState = inputState;
        addRequirements(lightIndicator);
    }

    @Override
        public void initialize() {
            //set the Servo position to the target - we only have to set the servo position one time
            lightIndicator.setStateLeft(targetState);
        }

        public void execute() {
            //nothing to do in the loop
        }

        @Override
        public boolean isFinished() {
            //always return true because the command simply sets the servo and we have no way of telling when the servo has finished moving
            return true;
        }

        @Override
        public void end(boolean interrupted) {
        }
    }

