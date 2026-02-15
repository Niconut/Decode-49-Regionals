package org.firstinspires.ftc.teamcode.subsystems.endgame;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;

@Configurable
public class Endgame_Kickstand extends SubsystemBase {
    private MyRobot robot;
    private Servo KickstandL;
    private Servo KickstandR;
    public static double[] DOWN = {0.7, 0.7};
    public static double[] UP = {0, 0};

    public enum EndgameState {
        DOWN,
        UP
    }

    public Endgame_Kickstand(MyRobot robot){
        this.robot = robot;
        KickstandL = robot.hardwareMap.get(Servo.class, "KickstandLeft");
        KickstandR = robot.hardwareMap.get(Servo.class, "KickstandRight");
        KickstandL.setDirection(Servo.Direction.REVERSE);
    }

    public void setPosition(double position){
        KickstandL.setPosition(position);
        KickstandR.setPosition(position);
    }

    public double getPositionLeft(){return KickstandL.getPosition();}
    public double getPositionRight(){return KickstandR.getPosition();}

    public void setState(EndgameState state){
        double[] vel = switch (state){
            case DOWN -> DOWN;
            case UP -> UP;
        };
        KickstandL.setPosition(vel[0]);
        KickstandR.setPosition(vel[1]);
    }
}
