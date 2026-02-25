package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class Constants {
    public static class FinalAutoTrajectories {
        public static final Vector2d robotEndPosBlue = new Vector2d(-34,-16);
    }

    public double DRIVE_NORMAL_SCALE = 1;
    public double STRAFE_NORMAL_SCALE = 1;
    public double ROT_NORMAL_SCALE = 0.5;

    public double DRIVE_SLOW_SCALE = 0.3;
    public double STRAFE_SLOW_SCALE = 0.3;
    public double ROT_SLOW_SCALE = 0.3;

    public double WRIST_MOVE_INCREMENTS = 0.02;
    public double WRIST_MOVE_THRESHOLD = 0.25;

    public double SHOULDER_MOVE_INCREMENTS = 0.01;
    public double SHOULDER_MOVE_THRESHOLD = 0.25;

    public double SLIDE_MOVE_INCREMENTS = 0.01;
    public double SLIDE_MOVE_THRESHOLD = 0.25;

    public double SCORING_ARM_MOVE_INCREMENTS = 0.01;
    public double SCORING_ARM_MOVE_THRESHOLD = 0.25;
}
