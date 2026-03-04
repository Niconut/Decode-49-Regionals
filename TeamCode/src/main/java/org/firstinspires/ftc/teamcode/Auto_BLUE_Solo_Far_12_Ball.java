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
@Autonomous (name = "Blue Solo Far 12", group = "Blue Alliance")
public class Auto_BLUE_Solo_Far_12_Ball extends LinearOpMode {

    private MecanumDrive drive;
    Action TrajectoryShootBallsAndPrep,
            TrajectoryBackPickUp,
            TrajectoryFrontPickUp,
            TrajectoryPark;


    public static Vector2d midPickupPrep = new Vector2d(14,-30);
    public static double midPickup = -64;
    public static Vector2d shootPrep1 = new Vector2d(12,-30);
    public static Vector2d secondScore = new Vector2d(56,-12);
    public static Vector2d frontPrep = new Vector2d(-11, -32);
    public static double frontPickup = -48;
    public static Vector2d fifthScorePrep = new Vector2d(-11,-32);
    public static Vector2d fifthScore = new Vector2d(56,-12);


    public void runOpMode() throws InterruptedException{
        Pose2d beginPose = new Pose2d(62, -12, Math.toRadians(-90));
        drive = new MecanumDrive(hardwareMap, beginPose);
        Scoring_Shooter_Action scoringShooter = new Scoring_Shooter_Action(hardwareMap);
        Intake_Subsystem_Action intakeSubsystem = new Intake_Subsystem_Action(hardwareMap);
        Distance_Sensor_Action distanceSensor = new Distance_Sensor_Action(hardwareMap);
        Scoring_Gate_Action scoringGate = new Scoring_Gate_Action(hardwareMap);
        Shooter_Subsystem_Action shooterSubsystem = new Shooter_Subsystem_Action(hardwareMap, Shooter_Subsystem_Action.Pipeline.BLUE                       );

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
                .lineToY(midPickup, new TranslationalVelConstraint(40))
                .splineToConstantHeading(shootPrep1, Math.toRadians(90), new TranslationalVelConstraint(70))
                .splineToConstantHeading(secondScore, Math.toRadians(0), new TranslationalVelConstraint(70));

        TrajectoryActionBuilder trajectoryFrontPickUp = trajectoryShootBallsandPickup.endTrajectory().fresh()
                .splineToConstantHeading(frontPrep, Math.toRadians(-90))
                .lineToY(frontPickup)
                .splineToConstantHeading(fifthScorePrep, Math.toRadians(90))
                .splineToConstantHeading(fifthScore, Math.toRadians(0));

        TrajectoryActionBuilder trajectoryBackPickUp = trajectoryFrontPickUp.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(36, -30), Math.toRadians(-90))
                .lineToY(-50)
                .splineToConstantHeading(new Vector2d(36,-46), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(56, -12), Math.toRadians(0));

        TrajectoryActionBuilder trajectoryPark = trajectoryBackPickUp.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(8, -40), Math.toRadians(-90));

        TrajectoryShootBallsAndPrep = trajectoryShootBallsandPickup.build();
        TrajectoryBackPickUp = trajectoryBackPickUp.build();
        TrajectoryFrontPickUp = trajectoryFrontPickUp.build();
        TrajectoryPark = trajectoryPark.build();
    }
    public Action scorePreload(Scoring_Shooter_Action scoringShooter,
                               Intake_Subsystem_Action intakeSubsystem,
                                Distance_Sensor_Action distanceSensor,
                                Scoring_Gate_Action scoringGate,
                               Shooter_Subsystem_Action shooterSubsystem){

        return new ParallelAction(
                new SequentialAction(
                        scoringGate.OpenGate(),
                        scoringShooter.FartherShooter(),
                        new SleepAction(3.5),
                        FAR_SHOOT(intakeSubsystem),
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
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(0.5),
                        scoringGate.CloseGate(),
                        // open the ramp gate

                        // shoot

                        // pickup 2nd set
                        scoringGate.CloseGate(),
                        new ParallelAction(
                                TrajectoryFrontPickUp,
                                INTAKE(intakeSubsystem, distanceSensor
                                )
                        ),
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(0.75),

                        // pickup 3rd set
                        scoringGate.CloseGate(),
                        new ParallelAction(
                                TrajectoryBackPickUp,
                                INTAKE(intakeSubsystem, distanceSensor)
                        ),
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(0.75),
                        // shoot
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

    public Action FAR_SHOOT(Intake_Subsystem_Action intakeSubsystem){
        return new SequentialAction(
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FARSHOOT)
        );
    }


}
