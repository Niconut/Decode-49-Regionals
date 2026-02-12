package org.firstinspires.ftc.teamcode.subsystems.Sensors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;

public class Light_Indicator extends SubsystemBase {
    private MyRobot robot;
    private Servo LightSensorLeft;

    private static double WHITE = 0.730;
    private static double PURPLE = 0.720;
    private static double BLUE = 0.618;
    private static double GREEN = 0.5;
    private static double YELLOW = 0.345;
    private static double RED = 0.280;
    private static double SAGE = 0.444;

    public enum LightIndicatorState {
        BLUE,
        GREEN,
        YELLOW,
        RED,
        PURPLE,
        WHITE,
        SAGE
    }

    public Light_Indicator(MyRobot robot) {
        this.robot = robot;
        this.LightSensorLeft = robot.hardwareMap.get(Servo.class, "LightLeft");
        //this.LightSensorRight = robot.hardwareMap.get(Servo.class, "LightRight");
    }

    public void setPosition(double position){
        LightSensorLeft.setPosition(position);
    }

    public void setStateLeft(LightIndicatorState state){
        double pos = switch (state){
            case RED -> RED;
            case BLUE -> BLUE;
            case GREEN -> GREEN;
            case YELLOW -> YELLOW;
            case SAGE -> SAGE;
            case WHITE -> WHITE;
            case PURPLE -> PURPLE;
        };
        LightSensorLeft.setPosition(pos);
    }

    /*public void setStateRight(LightIndicatorState state){
        double pos = switch (state){
            case RED -> RED;
            case BLUE -> BLUE;
            case GREEN -> GREEN;
            case YELLOW -> YELLOW;
            case SAGE -> SAGE;
            case WHITE -> WHITE;
            case PURPLE -> PURPLE;
        };
        LightSensorRight.setPosition(pos);
    }*/
}