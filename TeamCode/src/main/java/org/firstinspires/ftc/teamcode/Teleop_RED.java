package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;
@Disabled
@TeleOp(name = "RED", group = "AA_Drive_Code")
public class Teleop_RED extends CommandOpMode {

    @Override
    public void initialize(){
        MyRobot robot = new MyRobot(this, MyRobot.UsingDrive.YesDrive, MyRobot.TeleOpModeType.Robot, MyRobot.TeleOpMode.RED);
    }
}
