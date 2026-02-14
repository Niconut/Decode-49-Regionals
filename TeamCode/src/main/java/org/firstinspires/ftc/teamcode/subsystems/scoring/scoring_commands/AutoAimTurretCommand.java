package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Shooter_Subsystem;
@Configurable
public class AutoAimTurretCommand extends CommandBase {
    private MyRobot robot;
    public PIDController PID;
    private final Shooter_Subsystem scoringShooterSubsystem;
    public final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public static double height1 = 16;// height of limelight lens from the floort
    public static double height2 = 29.5;// height of center of apriltag to the floor
    public static double angle1 = 0; // mounting angle of the limelight degrees back from vertical
    public double distance = 0;
    public double shooterPower = 0;
    private static double turretProp[] = {0, 0};
    private static double turretBearing = 0;
    private static double turretRange = 0;
    private static double robotPos[] = {0,0};
    public static double kp = 0.03 ;
    public static double ki = 0.05 ;
    public static double kd = 0.0 ;
    public static double power = 0;
    public static double kf = 0.01 ;
    public static double targetAngle = 0;
    private static double TURRET_TARGET_POSITION = 0.5;
//    private static double TURRET_ANGLE_INCREMENTS = 0.0001;
    private static double TURRET_ANGLE_THRESHOLD = 0.5;
    private static double BIG_GEAR_TEETH = 100;
    private static double SMALL_GEAR_TEETH = 40;
    private static double SERVO_TRAVEL_ANGLE = 320;
    private static double SERVO_TRAVEL_VOLTAGE = 0.5;
    private static double SERVO_GAIN_FACTOR = 1;
    private static double TURRET_ANGLE_INCREMENTS = ((SERVO_TRAVEL_VOLTAGE/SERVO_TRAVEL_ANGLE)*(SMALL_GEAR_TEETH/BIG_GEAR_TEETH))/SERVO_GAIN_FACTOR;
    private static double FLYWHEEL_FILTER_COEFF_A = 50;
    private static double FLYWHEEL_FILTER_COEFF_B = 256;
    private static double flywheelFilteredRange = 0;

    private static double FLYWHEEL_SPEED_THRESHOLD = 25;

    private static double FLYWHEEL_RANGE_FACTOR = 13.5;
    private static double flywheelTargetSpeed = 0;
    private static double flywheelCurrentSpeed = 0;
    double counter = 0;
    double turretAngleThreshold = 1.50;
    public static double lowerPower = 1200;
    public static double higherPower = 1400;
    public static boolean startShooter = true;
    private AnalogInput turretAnalog; // Only source of position data
    private GoBildaPinpointDriver odo;


    public AutoAimTurretCommand(Shooter_Subsystem subsystem, double TargetAngle, MyRobot robot, boolean StartShooter){
        startShooter = StartShooter;
        this.robot = robot;
        targetAngle = TargetAngle;
        scoringShooterSubsystem = subsystem;
        PID = new PIDController(kp, ki, kd);
        addRequirements(scoringShooterSubsystem);
        odo = robot.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-3.75, -3.17, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
        if (scoringShooterSubsystem.getDetected()){
            robotPos = scoringShooterSubsystem.getRobotPosition();
            double x = robotPos[0];
            double y = robotPos[1];
            odo.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, 0));
        }
    }

    @Override
    public void initialize() {
        counter = 0;
        //scoringShooterSubsystem.resumeStraming();
    }

    public void execute() {
        //nothing to do in the loop

        PID.setPID(kp, ki, kd);
        turretProp = scoringShooterSubsystem.detectAprilTag();
        turretBearing = (turretProp[0]);
        turretRange = (turretProp[1]);
        double targettx = targetAngle;
        odo.update();



        Pose2D robotPose = odo.getPosition();
        double y = robotPose.getX(DistanceUnit.INCH);
        double x = robotPose.getY(DistanceUnit.INCH);
        double H = robotPose.getHeading(AngleUnit.RADIANS);
        double robotH = -(H);
        double RED_GOAL_X = 13.63;
        double RED_GOAL_Y = 127.64;

        double dx = RED_GOAL_X - x;
        double dy = RED_GOAL_Y - y;
        double floorDistance = Math.hypot(dx, dy);


        shooterPower = 975 + (floorDistance - 70) * ((1300.0 - 975.0) / (124.0 - 70.0));


        /*if ((distance <=72.9) && (distance >=60)){
            shooterPower = 1300;
        } else if ((distance <=59.9) && (distance >=45)){
            shooterPower = 1200;
        } else if ((distance <=80.5) && (distance >=73)){
            shooterPower = 1200;
        }else {
            shooterPower = 0;
        }*/



        /*if ((Math.abs(turretRange)!= 0) && startShooter) {
            distance = (height2 - height1) / Math.tan(Math.toRadians(angle1 + turretRange));
            if (robot.operator.getButton(GamepadKeys.Button.DPAD_UP)) {
                shooterPower = higherPower + (distance - 70) * ((1700.0 - 1300.0) / (124.0 - 70.0));
            } else if (robot.operator.getButton(GamepadKeys.Button.DPAD_DOWN)){
                shooterPower = lowerPower + (distance - 70) * ((1700.0 - 1300.0) / (124.0 - 70.0));
            } else {
                shooterPower = 1300 + (distance -70) * ((1700.0 - 1300.0) / (124.0 - 70.0));
            }
        } else if (!startShooter){
            shooterPower = 10;
        }*/
        scoringShooterSubsystem.setVelocity(shooterPower);

        double error = turretBearing - targettx;
        if((Math.abs(turretBearing) != 0)) {
            power = PID.calculate(turretBearing, targettx);
        } else {
            power = 0;
        }

        if ( (Math.abs(turretBearing) != 0) && (Math.abs(targetAngle - turretBearing) < turretAngleThreshold)){
            counter = counter + 1;
        }
        if (counter >= 5){
            scoringShooterSubsystem.lightGreen();
        } else {
            scoringShooterSubsystem.lightRed();
        }

        if (scoringShooterSubsystem.getDetected() == false || Math.abs(targetAngle - turretBearing) > turretAngleThreshold){
            counter = 0;
        }

        //kp * error;
        scoringShooterSubsystem.panelTelemetry(turretBearing, power, shooterPower, distance, robotPose);
        scoringShooterSubsystem.setTurretPower(power);

    }
}



