package org.firstinspires.ftc.teamcode.subsystems.gyro;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class gyroSubsystem extends SubsystemBase {
    public IMU imu;
    public double currentAbsoluteYawDegrees;
    public double currentAbsoluteYawRadians;

    private final RevHubOrientationOnRobot hubOrientation =
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, // direction of control hub logo on robot
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD); // direction of USB ports on robot


    public gyroSubsystem(final HardwareMap hMap, final String name) {
        imu = hMap.get(IMU.class, "imu");
    }

    public void gyroinit() {
        imu.initialize(new IMU.Parameters(hubOrientation));
        imu.resetYaw();
    }

    public void getYawValues() {
        YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
        currentAbsoluteYawDegrees = angle.getYaw(AngleUnit.DEGREES);
        currentAbsoluteYawRadians = angle.getYaw(AngleUnit.RADIANS);
    }
}
