package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.robot.MyRobot;

public class Intake_Indexer extends SubsystemBase {
    private MyRobot robot;
    public CRServo indexer;


    public double INIT = 0;
    public double UP = -1;
    public double DOWN = 1;

    public enum IntakeIndexerState{
        INIT,
        UP,
        DOWN
    }

    public Intake_Indexer(MyRobot robot){
        this.robot = robot;
        indexer = robot.hardwareMap.get(CRServo.class, "Indexer");

        this.indexer.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setState(IntakeIndexerState state){
        double power = switch (state){
            case INIT -> INIT;
            case UP -> UP;
            case DOWN -> DOWN;
        };
        indexer.setPower(power);
    }
}
