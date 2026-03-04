package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Actions.Distance_Sensor_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Actions.Intake_Subsystem_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Gate_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Shooter_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Shooter_Subsystem_Action;
import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;
@Configurable
@Disabled
@Autonomous (name = "Blue Far Auto", group = "Blue ALliance")
public class Blue_Far_12_Ball extends LinearOpMode {

    private MecanumDrive drive;
    Action TrajectoryShootBallsAndPrep,
            TrajectoryShootBalls2,
            TrajectoryShootBalls3,
            TrajectoryBackPickUp,
            TrajectoryShootBalls4,
            TrajectoryFrontPickUp,
            TrajectoryPark;


    public static Vector2d midPickupPrep = new Vector2d(14,-30);
    public static double midPickup = -65;
    public static Vector2d shootPrep1 = new Vector2d(12,-30);
    public static Vector2d secondScore = new Vector2d(56,-12);
    public static Vector2d frontPrep = new Vector2d(-11, -32);
    public static double frontPickup = -64;
    public static Vector2d fifthScorePrep = new Vector2d(-11,-32);
    public static Vector2d fifthScore = new Vector2d(56,-12);


    public void runOpMode() throws InterruptedException{
        Pose2d beginPose = new Pose2d(62, -12, Math.toRadians(-90));
        drive = new MecanumDrive(hardwareMap, beginPose);
        Scoring_Shooter_Action scoringShooter = new Scoring_Shooter_Action(hardwareMap);
        Intake_Subsystem_Action intakeSubsystem = new Intake_Subsystem_Action(hardwareMap);
        Distance_Sensor_Action distanceSensor = new Distance_Sensor_Action(hardwareMap);
        Scoring_Gate_Action scoringGate = new Scoring_Gate_Action(hardwareMap);
        Shooter_Subsystem_Action shooterSubsystem = new Shooter_Subsystem_Action(hardwareMap, Shooter_Subsystem_Action.Pipeline.BLUE);

        buildTrajectories(drive, beginPose);
        waitForStart();
        Actions.runBlocking(
                new SequentialAction(
                        scorePreload(scoringShooter, intakeSubsystem, distanceSensor, scoringGate, shooterSubsystem)
                )

        );
    }

    private void buildTrajectories(MecanumDrive drive, Pose2d beginPose){
        // preload and pickup mid
        TrajectoryActionBuilder trajectoryShootBallsandPickup = drive.actionBuilder(beginPose)
                .setReversed(false)
                .splineToConstantHeading(midPickupPrep, Math.toRadians(-90), new TranslationalVelConstraint(50))
                .lineToY(midPickup, new TranslationalVelConstraint(40));


        // shoot mid
        TrajectoryActionBuilder trajectoryShootBalls2 = trajectoryShootBallsandPickup.endTrajectory().fresh()
                .splineToConstantHeading(shootPrep1, Math.toRadians(90))
                .splineToConstantHeading(secondScore, Math.toRadians(0));

        TrajectoryActionBuilder trajectoryFrontPickUp = trajectoryShootBalls2.endTrajectory().fresh()
                .splineToConstantHeading(frontPrep, Math.toRadians(-90))
                .lineToY(frontPickup);

        TrajectoryActionBuilder trajectoryShootBalls3 = trajectoryFrontPickUp.endTrajectory().fresh()
                .splineToConstantHeading(fifthScorePrep, Math.toRadians(90))
                .splineToConstantHeading(fifthScore, Math.toRadians(0));

        TrajectoryActionBuilder trajectoryBackPickUp = trajectoryShootBalls3.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(36, -30), Math.toRadians(-90))
                .lineToY(-67);

        TrajectoryActionBuilder trajectoryShootBalls4 = trajectoryBackPickUp.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(36,-30), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56, -12), Math.toRadians(0));

        TrajectoryActionBuilder trajectoryPark = trajectoryShootBalls4.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(8, -50), Math.toRadians(-90));
        TrajectoryShootBallsAndPrep = trajectoryShootBallsandPickup.build();
        TrajectoryShootBalls2 = trajectoryShootBalls2.build();
        TrajectoryShootBalls3 = trajectoryShootBalls3.build();
        TrajectoryBackPickUp = trajectoryBackPickUp.build();
        TrajectoryShootBalls4= trajectoryShootBalls4.build();
        TrajectoryFrontPickUp = trajectoryFrontPickUp.build();
        TrajectoryPark = trajectoryPark.build();
    }

    public Action spinShooter(Scoring_Shooter_Action scoringShooter){
        return new SequentialAction(
                scoringShooter.ShootBalls()
        );
    }
    public Action scorePreload(Scoring_Shooter_Action scoringShooter,
                               Intake_Subsystem_Action intakeSubsystem,
                                Distance_Sensor_Action distanceSensor,
                                Scoring_Gate_Action scoringGate,
                               Shooter_Subsystem_Action shooterSubsystem){

        return new ParallelAction(
                new SequentialAction(
                scoringGate.OpenGate(),
                scoringShooter.FarShooter(),
                new SleepAction(2.5),
                SHOOT_REAR_MID_FRONT(intakeSubsystem),
                new SleepAction(0.5),
                // score preload then pickup first set of artifacts
                new ParallelAction(
                    TrajectoryShootBallsAndPrep,
                    new SequentialAction(
                        new SleepAction(0.75),
                        scoringGate.CloseGate(),
                        INTAKE(intakeSubsystem, distanceSensor)
                    )
                ),

                // open the ramp gate
               // TrajectoryGateBackUp,

                // shoot
                new ParallelAction(
                    TrajectoryShootBalls2,
                    scoringShooter.FarShooter()
                ),
                scoringGate.OpenGate(),
                new SleepAction(0.35),
                SHOOT_REAR_MID_FRONT(intakeSubsystem),
                new SleepAction(0.75),

                // pickup 2nd set
                scoringGate.CloseGate(),
                new ParallelAction(
                    TrajectoryFrontPickUp,
                    INTAKE(intakeSubsystem, distanceSensor
                    )
                ),

                // shoot
                new ParallelAction(
                    TrajectoryShootBalls3,
                    scoringShooter.FarShooter()
                ),
                scoringGate.OpenGate(),
                new SleepAction(0.35),
                SHOOT_REAR_MID_FRONT(intakeSubsystem),
                new SleepAction(0.75),

                // pickup 3rd set
                scoringGate.CloseGate(),
                new ParallelAction(
                    TrajectoryBackPickUp,
                    INTAKE(intakeSubsystem, distanceSensor)
                ),

                // shoot
                new ParallelAction(
                        TrajectoryShootBalls4,
                        scoringShooter.FarShooter()
                ),
                scoringGate.OpenGate(),
                new SleepAction(0.35),
                SHOOT_REAR_MID_FRONT(intakeSubsystem),
                new SleepAction(0.75),

                TrajectoryPark
                ),
                shooterSubsystem.AutoAim()
        );
    }

    public Action SHOOT_REAR_MID_FRONT(Intake_Subsystem_Action intakeSubsystem){
        return new SequentialAction(
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FARSHOOT)
        );
    }

    public Action INTAKE(Intake_Subsystem_Action intakeSubsystem, Distance_Sensor_Action distanceSensor){
        return new SequentialAction(
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FORWARD_FORWARD_REVERSE),
                distanceSensor.ShootTriggered(),
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FORWARD_FORWARD_OFF),
                distanceSensor.RearTriggered(),
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FORWARD_OFF_OFF),
                distanceSensor.FrontTriggered(),
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.INIT)
        );
    }



}
