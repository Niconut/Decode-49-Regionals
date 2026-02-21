package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;

@TeleOp(name = "Turret Testing", group = "AA_Drive_Code")
public class Turret_Testing extends CommandOpMode {

    @Override
    public void initialize(){
        MyRobot robot = new MyRobot(this, MyRobot.UsingDrive.NoDrive, MyRobot.TeleOpModeType.Robot, MyRobot.TeleOpMode.TurretTesting);
    }
}
