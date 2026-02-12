package org.firstinspires.ftc.teamcode.subsystems.drive;



import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;


public class driveSubsystem extends SubsystemBase {
    private final MecanumDrive drive;
    public IMU imu;
    double leftXAdj = 0;
    double leftYAdj = 0;
    public boolean fieldOriented = false;

    public driveSubsystem(HardwareMap hardwareMap, Pose2d Pose2d) {
        drive = new MecanumDrive(hardwareMap,
                new Pose2d(0,0,0));
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void setDrivePower(double leftX, double leftY, double rightX){
        if (fieldOriented) {
            drive.setFieldDrivePowers2(leftX, leftY, rightX);
        }else{
            leftXAdj = leftX;
            leftYAdj = leftY;
            drive.setDrivePowers((new PoseVelocity2d(new Vector2d(leftYAdj, -leftXAdj), -rightX)));
        }
    }

    public void setFieldOriented(boolean fieldOriented) {
        this.fieldOriented = fieldOriented;
    }

    public void fieldOrientedControl (double leftY, double leftX) {
        double y = leftY;
        double x = leftX;
        double botHeading;

        YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
        botHeading = angle.getYaw(AngleUnit.DEGREES);

        // Rotate the movement direction counter to the bot's rotation
        leftXAdj = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        leftYAdj = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        leftYAdj = Math.min(leftYAdj * 1.1, 1);  // Counteract imperfect strafing

    }
    public double getRawExternalHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
