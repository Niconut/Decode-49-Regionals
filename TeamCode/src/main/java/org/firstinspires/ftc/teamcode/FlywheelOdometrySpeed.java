package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.drive.*;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drive.driveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.driveCommands.SlowModeCommand;


public class FlywheelOdometrySpeed extends LinearOpMode{
    private GoBildaPinpointDriver odo;
    private driveSubsystem drive;
    public Command defaultDriveCommand;
    public Command slowModeCommand;
    private static final int TICKS_PER_REV = 28;
    private DcMotorEx flywheel;
    private double goalX = 0;
    private double goalY = 0;
    private double redGoalX = -55.64;
    private double redGoalY = 58.37;
    private double blueGoalX = -55.64;
    private double blueGoalY = -58.37;
    private double flywheelRobotPosX = 0;
    private double flywheelRobotPosY = 0;
    private double farDistanceSpeed = 1353;
    private double farDistanceSpeedBlue = 1550;
    private double farDistanceSpeedRed = 1540;
    private double targetDistance = 0;
    private double shooterPower = 0;
    private double closeDistanceSpeed = 1360;
    private double closeDistance = 80;
    private double farDistance = 118;
    private double minimumSpeed = 1250;
    private double maximumSpeed = 1650;
    public FlywheelOdometrySpeed(Team team) {
        this.team = team;
    }


    public enum Team {
        Blue, Red
    }

    private final Team team;
    @Override
    public void runOpMode(){
        GamepadEx driver = new GamepadEx(gamepad1);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        MakeCommands(driver);
        waitForStart();
        while (opModeIsActive()) {

            changeFlywheelSpeed();
        }

    }
    public void setSpeed(double RPM) {
        double TPS = (RPM * TICKS_PER_REV) / 60.0;
        flywheel.setVelocity(TPS);
    }
    public void changeFlywheelSpeed(){
        Pose2D robotPose = odo.getPosition();
        if (team == Team.Blue) {
            goalX = blueGoalX;
            goalY = blueGoalY;
            flywheelRobotPosX = robotPose.getX(DistanceUnit.INCH);
            flywheelRobotPosY = robotPose.getY(DistanceUnit.INCH);
            farDistanceSpeed = farDistanceSpeedBlue;
        }
        if(team == Team.Red) {
            goalX = redGoalX;
            goalY = redGoalY;
            flywheelRobotPosX = robotPose.getX(DistanceUnit.INCH);
            flywheelRobotPosY = robotPose.getY(DistanceUnit.INCH);
            farDistanceSpeed = farDistanceSpeedRed;
        }
        targetDistance = Math.hypot(goalX-flywheelRobotPosX, goalY-flywheelRobotPosY);
        shooterPower = closeDistanceSpeed+(targetDistance-closeDistance)*((farDistanceSpeed-closeDistanceSpeed)/(farDistance-closeDistance));
        shooterPower = Math.max(shooterPower, minimumSpeed);
        shooterPower = Math.min(shooterPower, maximumSpeed);
        setSpeed(shooterPower);
    }
    private void MakeCommands(GamepadEx gamepad) {
        defaultDriveCommand = new DefaultDriveCommand(drive,
                gamepad::getLeftX,
                gamepad::getLeftY,
                gamepad::getRightX
        );

        slowModeCommand = new SlowModeCommand(drive,
                gamepad::getLeftX,
                gamepad::getLeftY,
                gamepad::getRightX
        );
    }

}
