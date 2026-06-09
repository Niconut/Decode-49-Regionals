package org.firstinspires.ftc.teamcode.subsystems.Sensors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.subsystems.Prism.Color;
import org.firstinspires.ftc.teamcode.subsystems.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.subsystems.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;

public class LedStrips extends SubsystemBase {
    private MyRobot robot;
    private GoBildaPrismDriver ledStrip;
    private PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.WHITE);

    public enum LightIndicatorState {
        RED,
        SCARLETRED,
        PURPLE,
        GREEN,
        ORANGE
    }

    public LedStrips(MyRobot robot) {
        this.robot = robot;
        this.ledStrip = robot.hardwareMap.get(GoBildaPrismDriver.class, "LedStrips");
        this.solid.setBrightness(50);
        this.solid.setStartIndex(0);
        this.solid.setStopIndex(23);
        //this.LightSensorRight = robot.hardwareMap.get(Servo.class, "LightRight");
    }

//    public void setPosition(double color){
//        solid.setPrimaryColor();
//    }

    public void setState(LightIndicatorState state){
        switch (state) {
            case RED:
                {solid.setPrimaryColor(Color.RED);}
            case PURPLE:
                {solid.setPrimaryColor(Color.PURPLE);}
            case ORANGE:
                {solid.setPrimaryColor(Color.ORANGE);}
            case SCARLETRED:
                {solid.setPrimaryColor(255,50,0);}
            case GREEN:
            default:
                {solid.setPrimaryColor(0,100,4);}
        }
        ledStrip.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, solid);
        ledStrip.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, solid);
    }
}