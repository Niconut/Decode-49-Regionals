package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.subsystems.scoring.Shooter_Subsystem;
@Configurable
public class AutoAimTurretCommand extends CommandBase {
    public PIDController PID;
    private final Shooter_Subsystem scoringShooterSubsystem;
    public final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    private static double turretProp[] = {0, 0};
    private static double turretBearing = 0;
    private static double turretRange = 0;
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
    public AutoAimTurretCommand(Shooter_Subsystem subsystem, double TargetAngle){
        targetAngle = TargetAngle;
        scoringShooterSubsystem = subsystem;
        PID = new PIDController(kp, ki, kd);
        addRequirements(scoringShooterSubsystem);
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
        scoringShooterSubsystem.turretTelemetry(turretBearing, power);
        scoringShooterSubsystem.setPower(power);

    }
}



