package org.firstinspires.ftc.teamcode.subsystems.Sensors;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;

public class Distance_Sensor extends SubsystemBase {
    public MyRobot robot;
    public DistanceSensor shootSensor;
    public DistanceSensor rearSensor;
    public DistanceSensor frontSensor;

    public Distance_Sensor(MyRobot robot){
        this.robot = robot;
        shootSensor = robot.hardwareMap.get(DistanceSensor.class, "shoot_distance");
        rearSensor = robot.hardwareMap.get(DistanceSensor.class, "rear_distance");
        frontSensor = robot.hardwareMap.get(DistanceSensor.class, "front_distance");
    }

    public double getDistance(DistanceUnit distanceUnit){
        return shootSensor.getDistance(DistanceUnit.INCH);
    }

    public boolean shootSensorTriggered(){
        if ((shootSensor.getDistance(DistanceUnit.INCH) < 6.00)) return true;
        else return false;
    }

    public boolean frontSensorTriggered(){
        if ((frontSensor.getDistance(DistanceUnit.INCH) < 6.00)) return true;
        else return false;
    }
    public boolean rearSensorTriggered(){
        if ((rearSensor.getDistance(DistanceUnit.INCH) < 6.00)) return true;
        else return false;
    }



}
