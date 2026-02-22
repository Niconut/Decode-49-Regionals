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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Shooter_Subsystem;

import org.firstinspires.ftc.teamcode.subsystems.PostStorage;

import static java.lang.Math.atan2;

import java.util.function.DoubleSupplier;

@Configurable
public class AutoAimTurretCommand extends CommandBase {
    private MyRobot robot;
    private GoBildaPinpointDriver odo;
    private AnalogInput turretAnalog;

    private PIDController limelight_PID;
    private PIDController odo_PID;

    private final DoubleSupplier turretSupplier;
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
    public static double limelight_kp = 0.01 ;
    public static double limelight_ki = 0.05 ;
    public static double limelight_kd = 0.00 ;
    public static double odo_kp = 0.75 ;
    public static double odo_ki = 0.0 ;
    public static double odo_kd = 0.0 ;
    public static double power = 0;
    public static double kf = 0.01 ;
    public static double targetAngle = 0;
    double counter = 0;
    double TURRET_ANGLE_THRESHOLD = 1.50;
    public static boolean startShooter = true;

    private double CLOSE_DISTANCE = 80;
    private double FAR_DISTANCE = 118;
    private double CLOSE_DISTANCE_SPEED = 1350;
    private double MIN_SPEED = 1200;
    private double MAX_SPEED = 1650;
    private double FAR_DISTANCE_SPEED = 1600;
    private double targetDistance = 0;
    private double RED_GOAL_X = -55.64;
    private double RED_GOAL_Y = 58.37;
    private double BLUE_GOAL_X = -55.64;
    private double BLUE_GOAL_Y = -58.37;
    private double goal_X = 0;
    private double goal_Y = 0;
    private double robotPosX = 0;
    private double robotPosY = 0;
    private double turretRobotPosX = 0;
    private double turretRobotPosY = 0;
    private double flywheelRobotPosX = 0;
    private double flywheelRobotPosY = 0;
    private double TURRET_MANUAL_CONTROL_THRESHOLD = 0.1;
    private double TURRET_LIMELIGHT_CONTROL_THRESHOLD = 10;
    private double robotResetPosX = 0;
    private double robotResetPosY = 0;
    private double robotResetPosXBlue = 64;
    private double robotResetPosYBlue = 64;
    private double robotResetPosXRed = 64;
    private double robotResetPosYRed = -64;
    private double robotResetAngleBlue = 90;
    private double robotResetAngleRed = 270;
    private double robotResetAngle = 0;
    private double MAX_TURRET_ANGLE = 1.4;
    private double flywhhelResetPosAdjustX = 0;
    private double flywhhelResetPosAdjustY = 0;
    private double flywhhelResetPosAdjustXBlue = 144;
    private double flywhhelResetPosAdjustYBlue = 144;
    private double flywhhelResetPosAdjustXRed = 0;
    private double flywhhelResetPosAdjustYRed = 0;
    private double totalEncoderAngle = 0;
    private double delta = 0;

    private double targetRelativeAngle = 0;
    private double turretBearingError = 0;
    private boolean limelightOdoReset = false;
    private double LIMELIGHT_ODO_OFFSET_X = 72;
    private double LIMELIGHT_ODO_OFFSET_Y = 72;

    // Tracking
    private double currentEncoderAngle = 0;
    private double initEncoderAngle = 0;
    private double lastEncoderAngle = 0;
    private double targetFieldAngle = 0;
    private double totalTurretAngle = 0;
    private double targetTurretAngle = 0;
    private double robotHeading = 0;
    private int encoderRotations = 0;
    private double turretStartingAngle = 0;
    private double turretAngleOffset = 0;
    private boolean initStates = false;

    // Constants
    private final double MAX_VOLTAGE = 3.3;
    private final double TWO_PI = 2.0 * Math.PI;

    // Use 5.812 if that is your exact physical gear ratio.
    private final double GEAR_RATIO = 5.812;

    private Pose3D limelightPose;

    public enum Team {
        Blue, Red
    }

    private int i = 0;
    private final Team team;


    public AutoAimTurretCommand(Shooter_Subsystem subsystem, double TargetAngle, MyRobot robot, boolean StartShooter, DoubleSupplier turretMover, Team team){
        this.robot = robot;
        this.team = team;
        scoringShooterSubsystem = subsystem;
        startShooter = StartShooter;
        targetAngle = TargetAngle;
        turretSupplier = turretMover;

        addRequirements(scoringShooterSubsystem);
    }

    @Override
    public void initialize() {
        limelight_PID = new PIDController(limelight_kp, limelight_ki, limelight_kd);
        odo_PID = new PIDController(odo_kp, odo_ki, odo_kd);

        turretAnalog = robot.hardwareMap.get(AnalogInput.class, "turretFeedback");

        odo = robot.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-3.75, -3.17, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        counter = 0;

        if (team == Team.Blue) {
            robotResetAngle = robotResetAngleBlue;
        }
        if (team == Team.Red){
            robotResetAngle = robotResetAngleRed;
        }

        double X = PostStorage.currentPose.position.x;
        double Y = PostStorage.currentPose.position.y;

        //odo.setPosition(new Pose2D(DistanceUnit.INCH, robotResetPosX, robotResetPosY, AngleUnit.RADIANS, Math.toRadians(robotResetAngle)));
        odo.setPosition(new Pose2D(DistanceUnit.INCH, X, Y, AngleUnit.RADIANS, Math.toRadians(robotResetAngle)));
        odo.update();

        turretAngleOffset = totalEncoderAngle / GEAR_RATIO;
        initStates = true;

    }

    public void execute() {
        turretProp = scoringShooterSubsystem.detectAprilTag();
        turretBearing = (turretProp[0]);
        turretRange = (turretProp[1]);

        // * * * * Odometry based flywheel speed calculation * * * *
        odo.update();
        Pose2D robotPose = odo.getPosition();

        if (team == Team.Blue) {
            goal_X = BLUE_GOAL_X;
            goal_Y = BLUE_GOAL_Y;
            flywheelRobotPosX = robotPose.getX(DistanceUnit.INCH);
            flywheelRobotPosY = robotPose.getY(DistanceUnit.INCH);
            turretRobotPosX = robotPose.getX(DistanceUnit.INCH);
            turretRobotPosY = robotPose.getY(DistanceUnit.INCH);
            robotResetPosX = robotResetPosXBlue;
            robotResetPosY = robotResetPosYBlue;
            robotResetAngle = robotResetAngleBlue;
        }
        if (team == Team.Red){
            goal_X = RED_GOAL_X;
            goal_Y = RED_GOAL_Y;
            flywheelRobotPosX = robotPose.getX(DistanceUnit.INCH);
            flywheelRobotPosY = robotPose.getY(DistanceUnit.INCH);
            turretRobotPosX = robotPose.getX(DistanceUnit.INCH);
            turretRobotPosY = robotPose.getY(DistanceUnit.INCH);
            robotResetPosX = robotResetPosXRed;
            robotResetPosY = robotResetPosYRed;
            robotResetAngle = robotResetAngleRed;
        }

        targetDistance = Math.hypot(goal_X - flywheelRobotPosX, goal_Y - flywheelRobotPosY);

        shooterPower = CLOSE_DISTANCE_SPEED + (targetDistance - CLOSE_DISTANCE) * ((FAR_DISTANCE_SPEED - CLOSE_DISTANCE_SPEED) / (FAR_DISTANCE - CLOSE_DISTANCE));
        shooterPower = Math.max(shooterPower, MIN_SPEED);
        shooterPower = Math.min(shooterPower, MAX_SPEED);
        scoringShooterSubsystem.setVelocity(shooterPower);

        currentEncoderAngle = (turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI; // goes from 0 to 2 PI

        delta = currentEncoderAngle - lastEncoderAngle;
        lastEncoderAngle = currentEncoderAngle;

        if (delta < -Math.PI) {         // Wrapped forward
            encoderRotations++;
        }
        else if (delta > Math.PI){      // Wrapped backward
            encoderRotations--;
        }
        // Total radians the SERVO has spun
        totalEncoderAngle = (encoderRotations * TWO_PI) + currentEncoderAngle;
        totalTurretAngle = (totalEncoderAngle / GEAR_RATIO);// + Math.toRadians(robotResetAngle);
        targetTurretAngle = totalTurretAngle - turretAngleOffset;

        robotHeading =  robotPose.getHeading(AngleUnit.RADIANS);
        targetRelativeAngle =  Math.toRadians(robotResetAngle) + targetTurretAngle - robotHeading;
        while (targetRelativeAngle > Math.PI) targetRelativeAngle -= TWO_PI;
        while (targetRelativeAngle < -Math.PI) targetRelativeAngle += TWO_PI;

        //targetRelativeAngle = Math.max(-1.5, Math.min(1.5, targetRelativeAngle));

//        if(targetTurretAngle < -Math.PI){
//            targetTurretAngle = targetTurretAngle + TWO_PI;
//        }

        // --- STEP 3: CALCULATE TARGET ---

        targetFieldAngle = atan2(goal_X - turretRobotPosX , goal_Y - turretRobotPosY);

        while (targetRelativeAngle < -Math.PI){
            targetRelativeAngle = targetRelativeAngle + TWO_PI;
        }

        if (targetFieldAngle > MAX_TURRET_ANGLE){
            targetFieldAngle = targetFieldAngle - Math.PI;
        }
        if (targetFieldAngle < -MAX_TURRET_ANGLE){
            targetFieldAngle = targetFieldAngle + Math.PI;
        }


        // * * * * Limelight based flywheel speed calculation * * * *
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

        // * * * * Reset Robot position * * * *
        if (robot.driver.getButton(GamepadKeys.Button.BACK)){
            //odo.setPosition(new Pose2D(DistanceUnit.INCH, robotResetPosX, robotResetPosY, AngleUnit.RADIANS, Math.toRadians(robotResetAngle)));
            odo.update();
            turretAngleOffset = totalEncoderAngle / GEAR_RATIO;
            initStates = true;
        }

        if (robot.operator.getButton(GamepadKeys.Button.BACK)){
            odo.setPosition(new Pose2D(DistanceUnit.INCH, robotResetPosX, robotResetPosY, AngleUnit.RADIANS, Math.toRadians(robotResetAngle)));
            odo.update();
            turretAngleOffset = totalEncoderAngle / GEAR_RATIO;
            initStates = true;
        }

        // * * * * Calculate Turret Rotation * * * *
        // Limelight Method Priority 1
        if ((Math.abs(turretBearing) != 0) && (Math.abs(turretSupplier.getAsDouble()) <= TURRET_MANUAL_CONTROL_THRESHOLD)) {
            power = -limelight_PID.calculate(turretBearing, targetAngle);
        }
        // Manual Method Priority 2
        else if (Math.abs(turretSupplier.getAsDouble()) > TURRET_MANUAL_CONTROL_THRESHOLD) {
            power = turretSupplier.getAsDouble();
        }
        // Odometry Method Priority 3
        else {
            if (initStates){
                power = odo_PID.calculate(targetRelativeAngle, targetFieldAngle);
            }
            else{
                power = 0;
            }
        }

        //power = Math.max(-1, Math.min(1, power));
        if(targetTurretAngle > MAX_TURRET_ANGLE && power > 0){
            power = 0;
        }
        if(targetTurretAngle < -MAX_TURRET_ANGLE && power < 0){
            power = 0;
        }
        scoringShooterSubsystem.setTurretPower(power);

        // * * * * Light Indicators * * * *
        if ( (Math.abs(turretBearing) != 0) && (Math.abs(targetAngle - turretBearing) < TURRET_ANGLE_THRESHOLD)){
            counter = counter + 1;
        }
        if (counter >= 5){
            scoringShooterSubsystem.lightGreen();
        } else {
            scoringShooterSubsystem.lightRed();
        }

        if (!scoringShooterSubsystem.getDetected() || Math.abs(targetAngle - turretBearing) > TURRET_ANGLE_THRESHOLD){
            counter = 0;
        }

        // * * * * Update Telemetry * * * *
        scoringShooterSubsystem.panelTelemetry(turretBearing, power, shooterPower, targetDistance, robotPose, targetRelativeAngle, currentEncoderAngle,
                delta, totalEncoderAngle, lastEncoderAngle, totalTurretAngle, targetFieldAngle, turretAngleOffset, turretAngleOffset, targetTurretAngle);


    }
}



