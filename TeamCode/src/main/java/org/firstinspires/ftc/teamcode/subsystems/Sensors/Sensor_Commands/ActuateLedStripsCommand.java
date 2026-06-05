package org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.ftccommon.configuration.EditLynxModuleActivity;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Prism.Color;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.LedStrips;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Light_Indicator;

public class ActuateLedStripsCommand extends CommandBase {
        private final LedStrips ledStrips;
        private ElapsedTime runtime;

    public ActuateLedStripsCommand(LedStrips subsystem){
        runtime = new ElapsedTime();
        ledStrips = subsystem;
        addRequirements(ledStrips);
    }

    @Override
        public void initialize() {
            runtime.reset();
            //set the Servo position to the target - we only have to set the servo position one time
            ledStrips.setState(LedStrips.LightIndicatorState.GREEN);
        }

        public void execute() {
            runtime.startTime();
            if ((runtime.seconds() >= 60) && (runtime.seconds() <= 60.1)) { //1 Minute In, 1 Minutes Remaining
                //prism.clearAllAnimations();
                ledStrips.setState(LedStrips.LightIndicatorState.ORANGE);
            }
            if ((runtime.seconds() >= 95) && (runtime.seconds() <= 95.1)) { //Blip: 1 Minute 35 Seconds In, 25 Sec Remaining
                ledStrips.setState(LedStrips.LightIndicatorState.PURPLE);
            }
            if ((runtime.seconds() >= 95.5) && (runtime.seconds() <= 95.6)) { //25 Sec Still Remaining
                ledStrips.setState(LedStrips.LightIndicatorState.SCARLETRED);
            }
            if ((runtime.seconds() >= 100) && (runtime.seconds() < 120)) { //1 Minute 40 Seconds In, 20 Sec Remaining
                ledStrips.setState(LedStrips.LightIndicatorState.RED);
                ledStrips.setState(LedStrips.LightIndicatorState.PURPLE);
            }

//            if ((runtime.seconds() >= 100) && (runtime.seconds() < 100.5)) { //1 Minute 40 Seconds In, 20 Sec Remaining// 500ms delay
//                ledStrips.setState(LedStrips.LightIndicatorState.RED);
//            }
//            if ((runtime.seconds()>=100.5)&& (runtime.seconds()>101)){
//                ledStrips.setState(LedStrips.LightIndicatorState.PURPLE);
//            }
//            if ((runtime.seconds() >= 101) && (runtime.seconds() < 101.5)) { //1 Minute 40 Seconds In, 20 Sec Remaining// 500ms delay
//                ledStrips.setState(LedStrips.LightIndicatorState.RED);
//            }
//            if ((runtime.seconds()>=101.5)&& (runtime.seconds()>102)){
//                ledStrips.setState(LedStrips.LightIndicatorState.PURPLE);
//            }
//            if ((runtime.seconds() >= 102) && (runtime.seconds() < 102.5)) { //1 Minute 40 Seconds In, 20 Sec Remaining// 500ms delay
//                ledStrips.setState(LedStrips.LightIndicatorState.RED);
//            }
//            if ((runtime.seconds()>=102.5)&& (runtime.seconds()>103)){
//                ledStrips.setState(LedStrips.LightIndicatorState.PURPLE);
//            }
//            if ((runtime.seconds() >= 103) && (runtime.seconds() < 103.5)) { //1 Minute 40 Seconds In, 20 Sec Remaining// 500ms delay
//                ledStrips.setState(LedStrips.LightIndicatorState.RED);
//            }
//            if ((runtime.seconds()>=103.5)&& (runtime.seconds()>104)){
//                ledStrips.setState(LedStrips.LightIndicatorState.PURPLE);
//            }
            if ((runtime.seconds() > 121)) { //2 Minutes
                ledStrips.setState(LedStrips.LightIndicatorState.GREEN);
            }
            //nothing to do in the loop
        }

        @Override
        public boolean isFinished() {
            //always return true because the command simply sets the servo and we have no way of telling when the servo has finished moving
            return false;
        }

        @Override
        public void end(boolean interrupted) {
        }
    }

