package org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake_Indexer_Action {
    public CRServo Indexer;
    public double INIT = 0;
    public double UP = -1;
    public double DOWN = 1;
    public Intake_Indexer_Action(HardwareMap hardwareMap){
        this.Indexer = hardwareMap.get(CRServo.class, "Indexer");
    }

    public class indexerUp implements Action{
        public boolean initialized = false;
        public double SHOOT = 1350;


        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Indexer.setPower(UP);
                initialized = true;
               /* double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }

    public class stopIndexer implements Action{
        public boolean initialized = false;
        public double INIT = 0;
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Indexer.setPower(INIT);
                initialized = true;
               /* double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }

    public class reverseIndexer implements Action{
        public boolean initialized = false;
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                Indexer.setPower(DOWN);
                initialized = true;
               /* double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }



    public Action IndexerUp(){return new indexerUp();}
    public Action StopIndexer(){return new stopIndexer();}
    public Action ReverseIndexer(){return new reverseIndexer();}
}
