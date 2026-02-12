package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;

@TeleOp(name = "RED", group = "AA_Drive_Code")
public class Teleop_RED extends CommandOpMode {

    @Override
    public void initialize(){
        MyRobot robot = new MyRobot(this, MyRobot.TeleOpModeType.Robot, MyRobot.TeleOpMode.RED);
    }
}
