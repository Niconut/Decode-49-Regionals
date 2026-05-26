package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;
@Configurable
public class Intake_Subsystem extends SubsystemBase {
    private MyRobot robot;
    public DcMotor frontIntake;
    public DcMotor backIntake;
    public DcMotor midIntake;

    /*public double INIT[] = {0, 0, 0};
    public double FRONTINTAKE[] = {1, 1, 0};
    public double BACKINTAKE[] = {0, -1, -1};
    public double BACKPRESHOOT[] = {0, 0, -1};
    public double FRONTPRESHOOT[] = {1, 0, -1};
    public double MIDDLEPRESHOOT[] = {0,-1,0};
    public double FRONTMIDDLEPRESHOOT[] = {1,1,0};
    public double BACKMIDDLEPRESHOOT[] = {0,1,-1};
    public double SHOOT[] = {1, 1, -1};*/

    public static double[] INIT = {0, 0, 0};
    public static double[] OFF_OFF_SLOWREVERSE = {0, 0, -0.5};
    public static double [] HALFSHOOT_SHOOT_SHOOT = {0.5, 0.5, -0.5};
    public static double [] THREE_QUARTERS_SHOOT = {0.75, 0.75, -0.75};
    public static double [] CLOSESHOOT_SHOOT_SHOOT = {1.0, 1.0, -1.0};
    public static double[] OFF_SLOWFORWARD_SLOWREVERSE = {0, 0.75, -0.75};
    public static double[] OFF_OFF_FORWARD = {0, 0, 0.5};
    public static double[] OFF_SLOWFORWARD_REVERSE = {0, 1, -1.0};
    public static double[] FORWARD_SLOWFORWARD_OFF = {1.0, 0.5, 0};
    public static double[] SLOWFORWARD_SLOWFORWARD_OFF = {0.6, 0.75, 0};
    public static double[] OFF_OFF_REVERSE = {0, 0, -1.0};
    public static double[] OFF_FORWARD_OFF = {0, 1.0, 0};
    public static double[] OFF_FORWARD_FORWARD = {0, 1.0, 1.0};
    public static double[] OFF_FORWARD_REVERSE = {0, 1.0, -1.0};
    public static double[] OFF_REVERSE_OFF = {0, -1.0, 0};
    public static double[] OFF_REVERSE_FORWARD = {0, -1.0, 1.0};
    public static double[] OFF_REVERSE_REVERSE = {0, -1.0, -1.0};

    public static double[] FORWARD_OFF_OFF = {1.0, 0, 0};
    public static double[] FORWARD_OFF_FORWARD = {1.0, 0, 1.0};
    public static double[] FORWARD_OFF_REVERSE = {1.0, 0, -1.0};
    public static double[] FORWARD_FORWARD_OFF = {1.0, 1.0, 0};
    public static double[] FORWARD_FORWARD_FORWARD = {1.0, 1.0, 1.0};
    public static double[] FORWARD_FORWARD_REVERSE = {1.0, 1.0, -1.0};
    public static double[] FORWARD_REVERSE_OFF = {1.0, -1.0, 0};
    public static double[] FORWARD_REVERSE_FORWARD = {1.0, -1.0, 1.0};
    public static double[] FORWARD_REVERSE_REVERSE = {1.0, -1.0, -1.0};

    public static double[] REVERSE_OFF_OFF = {-1.0, 0, 0};
    public static double[] REVERSE_OFF_FORWARD = {-1.0, 0, 1.0};
    public static double[] REVERSE_OFF_REVERSE = {-1.0, 0, -1.0};
    public static double[] REVERSE_FORWARD_OFF = {-1.0, 1.0, 0};
    public static double[] REVERSE_FORWARD_FORWARD = {-1.0, 1.0, 1.0};
    public static double[] REVERSE_FORWARD_REVERSE = {-1.0, 1.0, -1.0};
    public static double[] REVERSE_REVERSE_OFF = {-1.0, -1.0, 0};
    public static double[] REVERSE_REVERSE_FORWARD = {-1.0, -1.0, 1.0};
    public static double[] REVERSE_REVERSE_REVERSE = {-1.0, -1.0, -1};
    public static double[] HALF_HALF_HALF = {0.75, 0.75, 0.75};


    public enum IntakeSubsystemState{
        INIT,
        HALFSHOOT_SHOOT_SHOOT,
        CLOSESHOOT_SHOOT_SHOOT,
        THREE_QUARTERS_SHOOT,
        OFF_OFF_FORWARD,
        OFF_OFF_REVERSE,
        OFF_SLOWFORWARD_REVERSE,
        OFF_SLOWFORWARD_SLOWREVERSE,
        OFF_OFF_SLOWREVERSE,
        FORWARD_SLOWFORWARD_OFF,
        SLOWFORWARD_SLOWFORWARD_OFF,
        OFF_FORWARD_OFF,
        OFF_FORWARD_FORWARD,
        OFF_FORWARD_REVERSE,
        OFF_REVERSE_OFF,
        OFF_REVERSE_FORWARD,
        OFF_REVERSE_REVERSE,
        FORWARD_OFF_OFF,
        FORWARD_OFF_FORWARD,
        FORWARD_OFF_REVERSE,
        FORWARD_FORWARD_OFF,
        FORWARD_FORWARD_FORWARD,
        FORWARD_FORWARD_REVERSE,
        FORWARD_REVERSE_OFF,
        FORWARD_REVERSE_FORWARD,
        FORWARD_REVERSE_REVERSE,
        REVERSE_OFF_OFF,
        REVERSE_OFF_FORWARD,
        REVERSE_OFF_REVERSE,
        REVERSE_FORWARD_OFF,
        REVERSE_FORWARD_FORWARD,
        REVERSE_FORWARD_REVERSE,
        REVERSE_REVERSE_OFF,
        REVERSE_REVERSE_FORWARD,
        REVERSE_REVERSE_REVERSE,
        HALF_HALF_HALF
    }
    public Intake_Subsystem(MyRobot robot){
        this.robot = robot;
        frontIntake = robot.hardwareMap.get(DcMotor.class, "frontIntake");
        backIntake = robot.hardwareMap.get(DcMotor.class, "backIntake");
        midIntake = robot.hardwareMap.get(DcMotor.class, "midIntake");

        this.frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.midIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setPower(double power){
        frontIntake.setPower(power);
        frontIntake.setPower(power);
        frontIntake.setPower(power);
    }

    public void setState(IntakeSubsystemState state){
        double power[] = switch (state){
            case INIT -> INIT;
            case HALFSHOOT_SHOOT_SHOOT -> HALFSHOOT_SHOOT_SHOOT;
            case CLOSESHOOT_SHOOT_SHOOT -> CLOSESHOOT_SHOOT_SHOOT;
            case THREE_QUARTERS_SHOOT -> THREE_QUARTERS_SHOOT;
            case OFF_OFF_FORWARD -> OFF_OFF_FORWARD;
            case OFF_OFF_REVERSE -> OFF_OFF_REVERSE;
            case OFF_OFF_SLOWREVERSE -> OFF_OFF_SLOWREVERSE;
            case OFF_SLOWFORWARD_REVERSE -> OFF_SLOWFORWARD_REVERSE;
            case OFF_SLOWFORWARD_SLOWREVERSE -> OFF_SLOWFORWARD_SLOWREVERSE;
            case FORWARD_SLOWFORWARD_OFF -> FORWARD_SLOWFORWARD_OFF;
            case SLOWFORWARD_SLOWFORWARD_OFF -> SLOWFORWARD_SLOWFORWARD_OFF;
            case OFF_FORWARD_OFF -> OFF_FORWARD_OFF;
            case OFF_FORWARD_FORWARD -> OFF_FORWARD_FORWARD;
            case OFF_FORWARD_REVERSE -> OFF_FORWARD_REVERSE;
            case OFF_REVERSE_OFF -> OFF_REVERSE_OFF;
            case OFF_REVERSE_FORWARD -> OFF_REVERSE_FORWARD;
            case OFF_REVERSE_REVERSE -> OFF_REVERSE_REVERSE;

            case FORWARD_OFF_OFF -> FORWARD_OFF_OFF;
            case FORWARD_OFF_FORWARD -> FORWARD_OFF_FORWARD;
            case FORWARD_OFF_REVERSE -> FORWARD_OFF_REVERSE;
            case FORWARD_FORWARD_OFF -> FORWARD_FORWARD_OFF;
            case FORWARD_FORWARD_FORWARD -> FORWARD_FORWARD_FORWARD;
            case FORWARD_FORWARD_REVERSE -> FORWARD_FORWARD_REVERSE;
            case FORWARD_REVERSE_OFF -> FORWARD_REVERSE_OFF;
            case FORWARD_REVERSE_FORWARD -> FORWARD_REVERSE_FORWARD;
            case FORWARD_REVERSE_REVERSE -> FORWARD_REVERSE_REVERSE;

            case REVERSE_OFF_OFF -> REVERSE_OFF_OFF;
            case REVERSE_OFF_FORWARD -> REVERSE_OFF_FORWARD;
            case REVERSE_OFF_REVERSE -> REVERSE_OFF_REVERSE;
            case REVERSE_FORWARD_OFF -> REVERSE_FORWARD_OFF;
            case REVERSE_FORWARD_FORWARD -> REVERSE_FORWARD_FORWARD;
            case REVERSE_FORWARD_REVERSE -> REVERSE_FORWARD_REVERSE;
            case REVERSE_REVERSE_OFF -> REVERSE_REVERSE_OFF;
            case REVERSE_REVERSE_FORWARD -> REVERSE_REVERSE_FORWARD;
            case REVERSE_REVERSE_REVERSE -> REVERSE_REVERSE_REVERSE;

            case HALF_HALF_HALF -> HALF_HALF_HALF;
        };
        frontIntake.setPower(power[0]);
        midIntake.setPower(power[1]);
        backIntake.setPower(power[2]);
    }


}
