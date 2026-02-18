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

import java.util.function.DoubleSupplier;

@Configurable
public class AutoAimTurretCommand extends CommandBase {
    private MyRobot robot;
    public PIDController PID;
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
    public static double kp = 0.03 ;
    public static double ki = 0.05 ;
    public static double kd = 0.0 ;
    public static double power = 0;
    public static double kf = 0.01 ;
    public static double targetAngle = 0;
    double counter = 0;
    double TURRET_ANGLE_THRESHOLD = 1.50;
    public static boolean startShooter = true;
    private AnalogInput turretAnalog; // Only source of position data
    private GoBildaPinpointDriver odo;
    private double CLOSE_DISTANCE = 80;
    private double FAR_DISTANCE = 118;
    private double CLOSE_DISTANCE_SPEED = 1315;
    private double MIN_SPEED = 1150;
    private double FAR_DISTANCE_SPEED = 1600;
    private double targetDistance = 0;
    private double RED_GOAL_X = 13.63;
    private double RED_GOAL_Y = 127.64;
    private double BLUE_GOAL_X = 13.63;
    private double BLUE_GOAL_Y = 127.64;
    private double goal_X = 0;
    private double goal_Y = 0;
    private double robotPosX = 0;
    private double robotPosY = 0;
    private double TURRET_MANUAL_CONTROL_THRESHOLD = 0.05;
    private double robotResetPosX = 72;
    private double robotResetPosY = 72;
    private double robotResetAngleBlue = 180;
    private double robotResetAngleRed = 0;
    private double robotResetAngle = 0;
    public enum Team {
        Blue, Red
    }

    private final Team team;


    public AutoAimTurretCommand(Shooter_Subsystem subsystem, double TargetAngle, MyRobot robot, boolean StartShooter, DoubleSupplier turretMover, Team team){
        this.team = team;
        startShooter = StartShooter;
        this.robot = robot;
        targetAngle = TargetAngle;
        scoringShooterSubsystem = subsystem;
        turretSupplier = turretMover;

        PID = new PIDController(kp, ki, kd);
        PID.setPID(kp, ki, kd);

        addRequirements(scoringShooterSubsystem);
        odo = robot.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-3.75, -3.17, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    @Override
    public void initialize() {
        counter = 0;
        odo.resetPosAndIMU();

        if (scoringShooterSubsystem.getDetected()){
            robotPos = scoringShooterSubsystem.getRobotPosition();
            odo.setPosition(new Pose2D(DistanceUnit.INCH, robotPos[0], robotPos[1], AngleUnit.RADIANS, Math.toRadians(0)));
        }
    }

    public void execute() {
        turretProp = scoringShooterSubsystem.detectAprilTag();
        turretBearing = (turretProp[0]);
        turretRange = (turretProp[1]);

        // * * * * Odometry based flywheel speed calculation * * * *
        odo.update();
        Pose2D robotPose = odo.getPosition();
        robotPosX = robotPose.getX(DistanceUnit.INCH);
        robotPosY = robotPose.getY(DistanceUnit.INCH);

        if (team == Team.Blue) {
            goal_X = BLUE_GOAL_X;
            goal_Y = BLUE_GOAL_Y;
            robotResetAngle = robotResetAngleBlue;
        }
        if (team == Team.Red){
            goal_X = RED_GOAL_X;
            goal_Y = RED_GOAL_Y;
            robotResetAngle = robotResetAngleRed;
        }
        targetDistance = Math.hypot(goal_X - robotPosX, goal_Y - robotPosY);

        shooterPower = CLOSE_DISTANCE_SPEED + (targetDistance - CLOSE_DISTANCE) * ((FAR_DISTANCE_SPEED - CLOSE_DISTANCE_SPEED) / (FAR_DISTANCE - CLOSE_DISTANCE));
        shooterPower = Math.max(shooterPower, MIN_SPEED);
        scoringShooterSubsystem.setVelocity(shooterPower);

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
                odo.setPosition(new Pose2D(DistanceUnit.INCH, robotResetPosX, robotResetPosY, AngleUnit.RADIANS, Math.toRadians(robotResetAngle)));
        }

        // * * * * Calculate Turret Rotation
        if((Math.abs(turretBearing) != 0) && (Math.abs(turretSupplier.getAsDouble()) <= TURRET_MANUAL_CONTROL_THRESHOLD)) {
            power = PID.calculate(turretBearing, targetAngle);
        } else if (turretSupplier.getAsDouble() != 0) {
            power = -turretSupplier.getAsDouble();
        } else {
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

        if (scoringShooterSubsystem.getDetected() == false || Math.abs(targetAngle - turretBearing) > TURRET_ANGLE_THRESHOLD){
            counter = 0;
        }

        // * * * * Update Telemetry * * * *
        scoringShooterSubsystem.panelTelemetry(turretBearing, power, shooterPower, targetDistance, robotPose);


    }
}



