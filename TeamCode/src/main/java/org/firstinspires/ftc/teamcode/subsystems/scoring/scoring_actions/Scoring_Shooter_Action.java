package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Configurable
public class Scoring_Shooter_Action {
    public DcMotorEx ScoringShooter;
    public static double SHOOT = 1350;
    public static double SECONDSHOOT = 1260;
    public static double FAR = 1600;
    public static double FARTHER = 1700;
    public Scoring_Shooter_Action(HardwareMap hardwareMap){
        ScoringShooter = hardwareMap.get(DcMotorEx.class, "ScoringShooter");
        this.ScoringShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.ScoringShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.ScoringShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.ScoringShooter.setVelocityPIDFCoefficients(100,0,0,0);
    }
    public class shootBalls implements Action{
        public boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                ScoringShooter.setVelocity(SHOOT);
                initialized = true;
            }
            return false;
        }
    }

    public class stopShooter implements Action{
        public boolean initialized = false;
        public double INIT = 0;
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                ScoringShooter.setVelocity(INIT);
                initialized = true;
               /* double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }

    public class reverseShooter implements Action{
        public boolean initialized = false;
        public double SLOW = -200;
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                ScoringShooter.setVelocity(SLOW);
                initialized = true;
               /* double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }

    public class farShooter implements Action{
        public boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                ScoringShooter.setVelocity(FAR);
                initialized = true;
               /* double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }

    public class fartherShooter implements Action{
        public boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                ScoringShooter.setVelocity(FARTHER);
                initialized = true;
               /* double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }



    public class closeShooter2 implements Action{
        public boolean initialized = false;

        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                ScoringShooter.setVelocity(SECONDSHOOT);
                initialized = true;
               /* double currentVel = Wheel1.getVelocity();
                initialized = (currentVel == targetvel);*/
            }

            return false;

        }
    }


    public Action ShootBalls(){return new shootBalls();}
    public Action StopShooter(){return new stopShooter();}
    public Action ReverseShooter(){return new reverseShooter();}
    public Action FarShooter(){return new farShooter();}
    public Action FartherShooter(){return new fartherShooter();}
    public Action CloseShooter2(){return new closeShooter2();}
}
