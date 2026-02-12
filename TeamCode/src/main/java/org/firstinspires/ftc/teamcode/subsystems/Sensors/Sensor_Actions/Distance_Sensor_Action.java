package org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Distance_Sensor_Action extends SubsystemBase {
    public DistanceSensor shootSensor;
    public DistanceSensor rearSensor;
    public DistanceSensor frontSensor;
    public ElapsedTime elapsedTime;

    public Distance_Sensor_Action(HardwareMap hardwareMap){
        shootSensor = hardwareMap.get(DistanceSensor.class, "shoot_distance");
        rearSensor = hardwareMap.get(DistanceSensor.class, "rear_distance");
        frontSensor = hardwareMap.get(DistanceSensor.class, "front_distance");
        elapsedTime = new ElapsedTime();
    }

    public double getDistance(DistanceUnit distanceUnit){
        return shootSensor.getDistance(DistanceUnit.INCH);
    }

    public boolean shootSensorTriggered(){
        if ((shootSensor.getDistance(DistanceUnit.INCH) < 5.00) || elapsedTime.seconds() > 1.75) return true;
        else return false;
    }

    public boolean frontSensorTriggered(){
        if ((frontSensor.getDistance(DistanceUnit.INCH) < 7.00) || elapsedTime.seconds() > 1.00) return true;
        else return false;
    }
    public boolean rearSensorTriggered(){
        if ((rearSensor.getDistance(DistanceUnit.INCH) < 5.00) || elapsedTime.seconds() > 1.25) return true;
        else return false;
    }
    public class shootTriggered implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized){
                    elapsedTime.reset();
                    initialized = true;
                }
                return !shootSensorTriggered();
            }
    }

    public class rearTriggered implements Action{
            private boolean initialized = false;
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if(!initialized){
                    elapsedTime.reset();
                    initialized = true;
                }
                return (!(shootSensorTriggered() && rearSensorTriggered()));
            }
    }
    public class frontTriggered implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized){
                elapsedTime.reset();
                initialized = true;
            }
            return (!(shootSensorTriggered() && rearSensorTriggered() && frontSensorTriggered()));
        }
    }

    public Action ShootTriggered(){return new shootTriggered();}
    public Action RearTriggered(){return new rearTriggered();}
    public Action FrontTriggered(){return new frontTriggered();}
}
