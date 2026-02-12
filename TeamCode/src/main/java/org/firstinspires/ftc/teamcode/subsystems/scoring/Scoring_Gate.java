package org.firstinspires.ftc.teamcode.subsystems.scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;

@Configurable
public class Scoring_Gate extends SubsystemBase {
    private MyRobot robot;
    private Servo GateServo;
    public static double CLOSE = 0.5;
    public static double OPEN = 0.36;

    public enum ScoringGState {
        CLOSE,
        OPEN
    }

    public Scoring_Gate(MyRobot robot){
        this.robot = robot;
        GateServo = robot.hardwareMap.get(Servo.class, "Gate");
    }

    public void setPosition(double position){
        GateServo.setPosition(position);
    }

    public double getPosition(){return GateServo.getPosition();}

    public void setState(ScoringGState state){
        double vel = switch (state){
            case CLOSE -> CLOSE;
            case OPEN -> OPEN;
        };
        GateServo.setPosition(vel);
    }
}
