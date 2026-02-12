  package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

  public class Shooter_Subsystem_Action extends SubsystemBase {
    public PIDController PID;
    private Limelight3A limelight;

    private CRServo servoTurret;
    public static double kp = 0.03 ;
    public static double ki = 0.05 ;
    public static double kd = 0.0 ;

    public enum Pipeline {
        RED,
        BLUE
    }

    public Shooter_Subsystem_Action(HardwareMap hardwareMap, Pipeline team){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        servoTurret = hardwareMap.get(CRServo.class, "ScoringTurret");    //  port 0
        servoTurret.setDirection(CRServo.Direction.FORWARD);
        PID = new PIDController(kp, ki, kd);
        if (team == Pipeline.RED){
            limelight.pipelineSwitch(1);
        } else if (team == Pipeline.BLUE){
            limelight.pipelineSwitch(0);
        }
    }

    public void setPower(double power){
        servoTurret.setPower(power);
    }



    public class autoAim implements Action{
        private boolean initialized = false;
        double turretBearing;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized){
                LLResult result = limelight.getLatestResult();

                double bearingAprilTag = result.getTx();
                double rangeAprilTag = result.getTy();

                double[] turretProp =  {bearingAprilTag, rangeAprilTag};
                turretBearing = (turretProp[0]);
                double turretRange = (turretProp[1]);

                double targettx = 0;

                double error = turretBearing - targettx;

                double power = PID.calculate(-turretBearing, targettx);
                //kp * error;
                servoTurret.setPower(power);
            }
            return true;
        }
    }

    public class INIT implements Action{
        public boolean initialized = false;
        public double INIT = 0;
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                servoTurret.setPower(INIT);
                initialized = true;
            }
            return false;
        }
    }

    public Action AutoAim(){return new autoAim();}
    public Action TurretINIT(){ return new INIT();}




}
