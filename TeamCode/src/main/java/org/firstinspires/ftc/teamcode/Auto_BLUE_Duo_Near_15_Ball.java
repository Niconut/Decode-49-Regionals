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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.PostStorage;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Actions.Distance_Sensor_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Actions.Intake_Subsystem_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Gate_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Shooter_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Shooter_Subsystem_Action;
import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;

@Configurable
@Autonomous (name = "BLUE Duo Near 15", group = "Blue Alliance")
public class Auto_BLUE_Duo_Near_15_Ball extends LinearOpMode {

    private MecanumDrive drive;
    Action TrajectoryShootBallsAndPrep,
            TrajectoryGateBackUp,
            TrajectoryShootBalls3,
            TrajectoryBackPickUp,
            TrajectoryFrontPickUp,
            TrajectoryPark;


    public static Vector2d firstScore = new Vector2d(0,-16);
    public static Vector2d midPickupPrep = new Vector2d(14,-30);
    public static double midPickup = -54;
    public static Vector2d shootPrep1 = new Vector2d(14,-30);
    public static Vector2d secondScore = new Vector2d(-16,-20);
    public static Vector2d frontPrep = new Vector2d(-11, -32);
    public static double frontPickup = -50;
    public static Vector2d fifthScorePrep = new Vector2d(-11,-30);
    public static Vector2d fifthScore = new Vector2d(-16,-20);


    public void runOpMode() throws InterruptedException{
        Pose2d beginPose = new Pose2d(-40, -57, Math.toRadians(-90));
        drive = new MecanumDrive(hardwareMap, beginPose);
        Scoring_Shooter_Action scoringShooter = new Scoring_Shooter_Action(hardwareMap);
        Intake_Subsystem_Action intakeSubsystem = new Intake_Subsystem_Action(hardwareMap);
        Distance_Sensor_Action distanceSensor = new Distance_Sensor_Action(hardwareMap);
        Scoring_Gate_Action scoringGate = new Scoring_Gate_Action(hardwareMap);
        Shooter_Subsystem_Action shooterSubsystem = new Shooter_Subsystem_Action(hardwareMap, Shooter_Subsystem_Action.Pipeline.BLUE);

        shooterSubsystem.setPower(0);
        buildTrajectories(drive, beginPose);
        waitForStart();
        PostStorage.currentPose = drive.localizer.getPose();
        Actions.runBlocking(
                new SequentialAction(
                        scorePreload(scoringShooter, intakeSubsystem, distanceSensor, scoringGate, shooterSubsystem)
                )

        );
        PostStorage.currentPose = drive.localizer.getPose();
    }

    private void buildTrajectories(MecanumDrive drive, Pose2d beginPose){
        // preload and pickup mid
        TrajectoryActionBuilder trajectoryShootBallsandPickup = drive.actionBuilder(beginPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(0,-16), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(15,-30), Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(13,-58), Math.toRadians(-90), new TranslationalVelConstraint(70))
                .splineToConstantHeading(new Vector2d(10,-46), Math.toRadians(-90), new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(0,-58), Math.toRadians(-90), new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(-4,-30), Math.toRadians(90), new TranslationalVelConstraint(70))
                .splineToConstantHeading(new Vector2d(-12,-20), Math.toRadians(-90), new TranslationalVelConstraint(70));

        TrajectoryActionBuilder trajectoryGateBackUp = trajectoryShootBallsandPickup.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-7, -32), Math.toRadians(-90), new TranslationalVelConstraint(30))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(-5,-58), Math.toRadians(-90), new TranslationalVelConstraint(70))
                .splineToConstantHeading(new Vector2d(-2,-48), Math.toRadians(-90), new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(4,-58), Math.toRadians(-90), new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(-16,-20), Math.toRadians(-90), new TranslationalVelConstraint(70));

        TrajectoryActionBuilder trajectoryShootBalls3 = trajectoryGateBackUp.endTrajectory().fresh()
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, -30), Math.toRadians(-90), new TranslationalVelConstraint(40))
                .lineToY(-60, new TranslationalVelConstraint(40))
                .splineToConstantHeading(new Vector2d(36,-46), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(10, -30), Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(-12, -20), Math.toRadians(-90));

        TrajectoryActionBuilder trajectoryFrontPickUp = trajectoryShootBalls3.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(30, -30), Math.toRadians(-90), new TranslationalVelConstraint(40))
                .splineToConstantHeading(new Vector2d(20,-62), Math.toRadians(-90), new TranslationalVelConstraint(70))
                .splineToConstantHeading(new Vector2d(16,-48), Math.toRadians(-90), new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(14,-62), Math.toRadians(-90), new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(-4,-30), Math.toRadians(90), new TranslationalVelConstraint(70))
                .splineToConstantHeading(new Vector2d(-12,-20), Math.toRadians(-90), new TranslationalVelConstraint(70));


        TrajectoryActionBuilder trajectoryBackPickUp = trajectoryFrontPickUp.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(8, -40), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryPark = trajectoryBackPickUp.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(9, -41), Math.toRadians(90));

        TrajectoryShootBallsAndPrep = trajectoryShootBallsandPickup.build();
        TrajectoryShootBalls3 = trajectoryShootBalls3.build();
        TrajectoryGateBackUp = trajectoryGateBackUp.build();
        TrajectoryBackPickUp = trajectoryBackPickUp.build();
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

        return
        new ParallelAction(
                    new SequentialAction(
                    scoringGate.OpenGate(),
                    scoringShooter.ShootBalls(),
                    new SleepAction(0.5),

                    // score preload then pickup first set of artifacts
                    new ParallelAction(
                        TrajectoryShootBallsAndPrep,
                        new SequentialAction(
                            new SleepAction(0.75),
                            SHOOT_REAR_MID_FRONT(intakeSubsystem),
                            new SleepAction(0.75),
                            scoringGate.CloseGate(),
                            scoringShooter.CloseShooter2(),
                            INTAKE(intakeSubsystem, distanceSensor)
                        )
                    ),
                    // shoot
                    scoringGate.OpenGate(),
                    new SleepAction(0.35),
                    SHOOT_REAR_MID_FRONT(intakeSubsystem),
                    new SleepAction(0.5),
                    scoringGate.CloseGate(),
                    new ParallelAction(
                            TrajectoryGateBackUp,
                            new SequentialAction(
                                    new SleepAction(0.5),
                                FAST_INTAKE(intakeSubsystem, distanceSensor)
                            )
                    ),
                    // shoot
                    scoringGate.OpenGate(),
                    new SleepAction(0.35),
                    SHOOT_REAR_MID_FRONT(intakeSubsystem),
                    new SleepAction(0.5),
                    scoringGate.CloseGate(),
                    new ParallelAction(
                            TrajectoryShootBalls3,
                            scoringShooter.CloseShooter2()
                    ),
                    scoringGate.OpenGate(),
                    new SleepAction(0.35),
                    SHOOT_REAR_MID_FRONT(intakeSubsystem),
                    new SleepAction(0.5),
                    // pickup 2nd set
                    scoringGate.CloseGate(),
                    new ParallelAction(
                        TrajectoryFrontPickUp,
                        FAST_INTAKE(intakeSubsystem, distanceSensor)
                    ),
                    scoringGate.OpenGate(),
                    new SleepAction(0.35),
                    SHOOT_REAR_MID_FRONT(intakeSubsystem),
                    new SleepAction(0.5),
                    // pickup 3rd set
                    scoringGate.CloseGate(),
                    TrajectoryBackPickUp,
                    TrajectoryPark
                ),
                shooterSubsystem.AutoAim()
        );
    }

    public Action SHOOT_REAR_MID_FRONT(Intake_Subsystem_Action intakeSubsystem){
        return new SequentialAction(
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FORWARD_FORWARD_REVERSE)
        );
    }

    public Action FAR_SHOOT(Intake_Subsystem_Action intakeSubsystem){
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
    public Action FAST_INTAKE(Intake_Subsystem_Action intakeSubsystem, Distance_Sensor_Action distanceSensor){
        return new SequentialAction(
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FORWARD_FORWARD_REVERSE),
                distanceSensor.ShootTriggered(),
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FASTFORWARD_FASTFORWARD_OFF),
                distanceSensor.RearTriggered(),
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FORWARD_OFF_OFF),
                distanceSensor.FrontTriggered(),
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.INIT)
        );
    }

    public Action SLOW_INTAKE(Intake_Subsystem_Action intakeSubsystem, Distance_Sensor_Action distanceSensor){
        return new SequentialAction(
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FORWARD_FORWARD_REVERSE),
                distanceSensor.ShootTriggered(),
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.SLOWFORWARD_SLOWFORWARD_OFF),
                distanceSensor.RearTriggered(),
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FORWARD_OFF_OFF),
                distanceSensor.FrontTriggered(),
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.INIT)
        );
    }




}
