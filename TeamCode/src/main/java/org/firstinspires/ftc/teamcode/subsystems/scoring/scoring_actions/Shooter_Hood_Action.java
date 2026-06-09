  package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands.AutoAimTurretCommand;

import java.nio.channels.Pipe;

  public class Shooter_Hood_Action extends SubsystemBase {

      public static double HOOD_CLOSE_POS = 0.55;
      public static double HOOD_FAR_POS = 0.42;
      public static double MIN_POS = 0.42;
      public static double MAX_POS = 0.8;
      private double hoodPos;
      private Servo servoHood;
      private GoBildaPinpointDriver odo;
      private double hoodRobotPosX;
      private double hoodRobotPosY;
      private double targetDistance = 0;
      private double RED_GOAL_X = -55.64;
      private double RED_GOAL_Y = 58.37;
      private double BLUE_GOAL_X = -55.64;
      private double BLUE_GOAL_Y = -58.37;
      private double goal_X = 0;
      private double goal_Y = 0;
      private double CLOSE_DISTANCE = 80;
      private double FAR_DISTANCE = 118;
      public enum Pipeline {
          RED,
          BLUE
      }

      private final Pipeline team;

    public Shooter_Hood_Action(HardwareMap hardwareMap, Pipeline team){
        this.team = team;
        servoHood = hardwareMap.get(Servo.class, "ScoringHood");
        servoHood.setDirection(Servo.Direction.FORWARD);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        odo.setOffsets(-3.75, -3.17, DistanceUnit.INCH);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        if (team == Pipeline.BLUE) {
            goal_X = BLUE_GOAL_X;
            goal_Y = BLUE_GOAL_Y;
        }
        if (team == Pipeline.RED){
            goal_X = RED_GOAL_X;
            goal_Y = RED_GOAL_Y;
        }
    }




    public class autoHood implements Action{
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            odo.update();
            Pose2D robotPose = odo.getPosition();

            hoodRobotPosX = robotPose.getX(DistanceUnit.INCH);
            hoodRobotPosY = robotPose.getY(DistanceUnit.INCH);
            // * * * * Hood Pos Calculation  * * * *
            // * * * * * * * * * * * * * * * * * * * * * *
            targetDistance = Math.hypot(goal_X - hoodRobotPosX, goal_Y - hoodRobotPosY);

            hoodPos = HOOD_CLOSE_POS + (targetDistance - CLOSE_DISTANCE) * ((HOOD_FAR_POS - HOOD_CLOSE_POS) / (FAR_DISTANCE - CLOSE_DISTANCE));
            hoodPos = Math.max(hoodPos, MIN_POS);
            hoodPos = Math.min(hoodPos, MAX_POS);
            servoHood.setPosition(hoodPos);
            return true;
        }
    }



    public Action AutoHood(){return new autoHood();}



}
