package org.firstinspires.ftc.teamcode.subsystems.scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;
import com.bylazar.configurables.annotations.Configurable;
@Configurable
public class Scoring_Shooter extends SubsystemBase {
    private MyRobot robot;
    private DcMotorEx ScoringShooter;
    public Telemetry telemetry;
    public static double SHOOT = 1300;//1425;
    public static double NEARNEARSHOOT = 1200;
    public static double FAR = 1625;
    public static double UNSTUCK = -500;
    public static double INTAKE_BACK_SPIN = -20;
    public static double INIT = 0;
    public static double kp = 150;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;

    public enum ScoringShooterState{
        INIT,
        SHOOT,
        UNSTUCK,
        INTAKE_BACK_SPIN,
        FAR,
        NEARNEARSHOOT
    }

    public Scoring_Shooter (MyRobot robot){
        this.robot = robot;
        this.telemetry = robot.telemetry;
        ScoringShooter = robot.hardwareMap.get(DcMotorEx.class, "ScoringShooter");
        this.ScoringShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.ScoringShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.ScoringShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.ScoringShooter.setVelocityPIDFCoefficients(kp,ki,kd,kf);
        this.ScoringShooter.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setVelocity(double vel){
        ScoringShooter.setVelocity(vel);
    }

    public double getVelocity(){
        return ScoringShooter.getVelocity();
    }


    public void shooterTelemetry(){
        telemetry.addData("Power", ScoringShooter.getVelocity());
        telemetry.update();
    }



    public void setState(ScoringShooterState state){
        double vel = switch (state){
            case INIT -> INIT;
            case SHOOT -> SHOOT;
            case UNSTUCK -> UNSTUCK;
            case INTAKE_BACK_SPIN -> INTAKE_BACK_SPIN;
            case FAR -> FAR;
            case NEARNEARSHOOT -> NEARNEARSHOOT;
        };

        ScoringShooter.setVelocity(vel);
    }
}
