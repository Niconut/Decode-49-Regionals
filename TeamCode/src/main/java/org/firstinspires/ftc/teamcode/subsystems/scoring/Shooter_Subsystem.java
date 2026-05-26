package org.firstinspires.ftc.teamcode.subsystems.scoring;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Shooter_Subsystem extends SubsystemBase {
    //private DcMotorEx ScoringShooter;
    private MyRobot robot;
    public Telemetry telemetry;
    public IMU imu;
    public final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private Servo LightRight;
    private Servo servoHood;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Limelight3A limelight;

    private CRServo servoTurret;

    private double SERVO_TURRET_INIT = 0;
    private double SERVO_TURRET_LEFT = 0.03;
    private double SERVO_TURRET_RIGHT = 0.97;

    private double SERVO_TURRET_SAFE_MIN = 0.0;
    private double SERVO_TURRET_SAFE_MAX = 1.0;
    private DcMotorEx ScoringShooter;
    public static double SHOOT = 1300;//1425;
    public static double NEARNEARSHOOT = 1200;
    public static double FAR = 1625;
    public static double UNSTUCK = -500;
    public static double INTAKE_BACK_SPIN = -20;
    public static double INIT = 0;
    public static double kp = 150;// 170
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;// 0
    public static double botX = 0;
    public static double botY = 0;
    public static double botHeading = 0;
    public enum Team {
        RED,
        BLUE
    }

    public Shooter_Subsystem(MyRobot robot, Team team) {
        limelight = robot.hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        servoTurret = robot.hardwareMap.get(CRServo.class, "ScoringTurret");    //  port 0
        servoTurret.setDirection(CRServo.Direction.FORWARD);
        servoHood = robot.hardwareMap.get(Servo.class, "ScoringHood");
        servoHood.setDirection(Servo.Direction.FORWARD);
        LightRight = robot.hardwareMap.get(Servo.class, "LightRight");
        ScoringShooter = robot.hardwareMap.get(DcMotorEx.class, "ScoringShooter");
        this.ScoringShooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.ScoringShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.ScoringShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.ScoringShooter.setVelocityPIDFCoefficients(kp,ki,kd,kf);
        this.ScoringShooter.setDirection(DcMotorSimple.Direction.FORWARD);
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

    public double[] getRobotPosition(){
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                double x = botpose.getPosition().x;
                double y = botpose.getPosition().y;
                botX = x;
                botY = y;
                telemetry.addData("MT1 Location", "(" + x + ", " + y + ")");
                telemetry.update();
            }
        }
        double[] botPos = {botX, botY};
        return botPos;
    }

    public boolean lightTreshold(){
        LLResult result = limelight.getLatestResult();
        double bearingAprilTag = result.getTx();
        return bearingAprilTag < 1.00 && bearingAprilTag > -1.00;
    }

    public Pose3D getLimelightPose(){
        LLResult result = limelight.getLatestResult();
        return result.getBotpose();
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
    public void setHoodPosition(double position){servoHood.setPosition(position);}
    public void setTurretPower(double power) {
        servoTurret.setPower(power);
    }
    public void setVelocity(double vel){
        double currentVel = ScoringShooter.getVelocity();
        double kp_adj = ((vel + 0.1 * (vel - currentVel))/vel) * kp;
        ScoringShooter.setVelocityPIDFCoefficients(kp_adj, ki, kd, kf);
        ScoringShooter.setVelocity(vel);
    }

    public double getVelocity(){
        return ScoringShooter.getVelocity();
    }


    public void shooterTelemetry(){
        telemetry.addData("Power", ScoringShooter.getVelocity());
        telemetry.update();
    }

    public void turretTelemetry(double bearing, double power){
        telemetry.addData("targetPower", power);
        telemetry.addData("targetBearing", bearing);
        telemetry.update();
    }

    public void panelTelemetry(
            double turretBearing,
            double power,
            double shooterPower,
            double targetDistance,
            Pose2D  robotPose,
            double targetRelativeAngle,
            double currentEncoderAngle,
            double delta,
            double totalEncoderAngle,
            double lastEncoderAngle,
            double totalTurretAngle,
            double targetFieldAngle,
            double turretAbsoluteAngle,
            double targetTurretAngle,
            double robotResetAngle,
            double calTurretRelativeAngleOffset,
            double maxTurretnAngleLimitRight,
            double turretZeroOffsetAngle,
            Pose2d autoEndPosition,
            double turretEndAngle
){
        panelsTelemetry.addData("targetPower", power);
        panelsTelemetry.addData("targetBearing", turretBearing);
        panelsTelemetry.addData("shooterPower", shooterPower);
        panelsTelemetry.addData("targetDistance", targetDistance);
        telemetry.addData("targetPower", power);
        telemetry.addData("targetBearing", turretBearing);
        telemetry.addData("shooterPower", shooterPower);
        telemetry.addData("Distance", targetDistance);
        telemetry.addData("PosX", servoHood.getPosition());
        telemetry.addData("PosX", robotPose.getX(DistanceUnit.INCH));
        telemetry.addData("PosY", robotPose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading", robotPose.getHeading(AngleUnit.RADIANS));
        telemetry.addData("robotRestAngle", robotResetAngle);
        telemetry.addData("lastEncoderAngle", lastEncoderAngle);
        telemetry.addData("currentEncoderAngle", currentEncoderAngle);
        telemetry.addData("delta", delta);
        telemetry.addData("totalEncoderAngle", totalEncoderAngle);
        telemetry.addData("totalTurretAngle", totalTurretAngle);
        telemetry.addData("targetFieldAngle", targetFieldAngle);
        telemetry.addData("targetTurretAngle", targetTurretAngle);
        telemetry.addData("targetRelativeAngle", targetRelativeAngle);
        telemetry.addData("turretZeroOffsetAngle", turretZeroOffsetAngle);
        telemetry.addData("calTurretRelativeAngleOffset", calTurretRelativeAngleOffset);
        telemetry.addData("turretAbsoluteAngle", turretAbsoluteAngle);
        telemetry.addData("autoEndPosition", autoEndPosition);
        telemetry.addData("turretAutoEndAngle", turretEndAngle);

        telemetry.update();
        panelsTelemetry.update();
    }

    public void setShooterPID(){
        this.ScoringShooter.setVelocityPIDFCoefficients(kp,ki,kd,kf);
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
