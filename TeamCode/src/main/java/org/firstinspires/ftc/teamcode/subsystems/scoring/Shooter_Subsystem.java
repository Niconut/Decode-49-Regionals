package org.firstinspires.ftc.teamcode.subsystems.scoring;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Shooter_Subsystem extends SubsystemBase {
    //private DcMotorEx ScoringShooter;
    private MyRobot robot;
    public Telemetry telemetry;
    private Servo LightRight;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Limelight3A limelight;

    private CRServo servoTurret;

    private double SERVO_TURRET_INIT = 0;
    private double SERVO_TURRET_LEFT = 0.03;
    private double SERVO_TURRET_RIGHT = 0.97;

    private double SERVO_TURRET_SAFE_MIN = 0.0;
    private double SERVO_TURRET_SAFE_MAX = 1.0;

    public enum Team {
        RED,
        BLUE
    }

    public Shooter_Subsystem(MyRobot robot, Team team) {
        limelight = robot.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        servoTurret = robot.hardwareMap.get(CRServo.class, "ScoringTurret");    //  port 0
        servoTurret.setDirection(CRServo.Direction.REVERSE);
        LightRight = robot.hardwareMap.get(Servo.class, "LightRight");
        this.telemetry = robot.telemetry;
        if (team == Team.RED){
            limelight.pipelineSwitch(1);
        } else if (team == Team.BLUE){
            limelight.pipelineSwitch(0);
        }
    }

    public double[] detectAprilTag() {
        LLResult result = limelight.getLatestResult();

        double bearingAprilTag = result.getTx();
        double rangeAprilTag = result.getTy();

        double[] propAprilTag =  {bearingAprilTag, rangeAprilTag};
        return propAprilTag;
    }

    public boolean lightTreshold(){
        LLResult result = limelight.getLatestResult();
        double bearingAprilTag = result.getTx();
        return bearingAprilTag < 1.00 && bearingAprilTag > -1.00;
    }

    public void stopStreaming(){
        visionPortal.stopStreaming();
    }

    public void resumeStraming(){
        visionPortal.resumeStreaming();
    }

    public void closePortal(){
        visionPortal.close();
    }

    public void setPower(double power) {
        servoTurret.setPower(power);
    }



    public void turretTelemetry(double bearing, double power){
        telemetry.addData("targetPower", power);
        telemetry.addData("targetBearing", bearing);
        telemetry.update();
    }



    public boolean getDetected(){
        LLResult result = limelight.getLatestResult();
        if (result != null){
            return true;
        }
        else return false;
    }
    public double getCurrentPosition(){return servoTurret.getPower();}



    public void lightGreen(){
        LightRight.setPosition(0.5);
    }

    public void lightRed(){
        LightRight.setPosition(0.280);
    }
}
