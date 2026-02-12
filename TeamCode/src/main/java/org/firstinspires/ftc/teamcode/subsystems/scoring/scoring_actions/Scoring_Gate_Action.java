package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
public class Scoring_Gate_Action {
    public Servo ScoringGate;
    public static double OPEN = 0.36;
    public static double CLOSE = 0.5;
    public Scoring_Gate_Action(HardwareMap hardwareMap){
        ScoringGate = hardwareMap.get(Servo.class, "Gate");
    }
    public class openGate implements Action{
        public boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                ScoringGate.setPosition(OPEN);
                initialized = true;
            }
            return false;
        }
    }

    public class closeGate implements Action{
        public boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                ScoringGate.setPosition(CLOSE);
                initialized = true;
            }
            return false;
        }
    }
    public Action OpenGate(){return new openGate();}
    public Action CloseGate(){return new closeGate();}
}
