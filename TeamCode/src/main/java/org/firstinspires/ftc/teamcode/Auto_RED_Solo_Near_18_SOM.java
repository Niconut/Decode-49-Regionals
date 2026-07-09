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

import org.firstinspires.ftc.teamcode.subsystems.Constants;
import org.firstinspires.ftc.teamcode.subsystems.PostStorage;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Actions.Distance_Sensor_Action;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Actions.Odom_Storage_Action;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Actions.Turret_Analog_Input_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Actions.Intake_Subsystem_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Gate_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Shooter_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Shooter_Hood_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Shooter_Subsystem_Action;
import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;

@Configurable
@Autonomous (name = "RED Solo Near 18", group = "Red Alliance")
public class Auto_RED_Solo_Near_18_SOM extends LinearOpMode {

    private MecanumDrive drive;
    Action TrajectoryShootBallsAndPrep,
            TrajectoryGateBackUp,
            TrajectoryShootBalls3,
            TrajectoryGateBackUp2,
            TrajectoryShootBalls4,
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
        Pose2d beginPose = new Pose2d(-40, 57, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, beginPose);
        Scoring_Shooter_Action scoringShooter = new Scoring_Shooter_Action(hardwareMap);
        Intake_Subsystem_Action intakeSubsystem = new Intake_Subsystem_Action(hardwareMap);
        Distance_Sensor_Action distanceSensor = new Distance_Sensor_Action(hardwareMap);
        Scoring_Gate_Action scoringGate = new Scoring_Gate_Action(hardwareMap);
        Shooter_Subsystem_Action shooterSubsystem = new Shooter_Subsystem_Action(hardwareMap, Shooter_Subsystem_Action.Pipeline.RED);
        Turret_Analog_Input_Action turretAnalog = new Turret_Analog_Input_Action(hardwareMap);
        Odom_Storage_Action odomStorage = new Odom_Storage_Action(hardwareMap, drive);
        Shooter_Hood_Action shooterHood = new Shooter_Hood_Action(hardwareMap, Shooter_Hood_Action.Pipeline.RED);


        shooterSubsystem.setPower(0);
        buildTrajectories(drive, beginPose);

        while (!isStarted()){
            Actions.runBlocking(
                    new SequentialAction(READTURRETPOSITION(shooterSubsystem)
                    )
            );
        }

        waitForStart();
        PostStorage.currentPose = drive.localizer.getPose();
        Actions.runBlocking(
                new SequentialAction(
                        scorePreload(scoringShooter, intakeSubsystem, distanceSensor, scoringGate, shooterSubsystem, turretAnalog, odomStorage, shooterHood)
                )

        );
        PostStorage.currentPose = drive.localizer.getPose();
    }

    private void buildTrajectories(MecanumDrive drive, Pose2d beginPose){
        // preload and pickup mid
        TrajectoryActionBuilder trajectoryShootBallsandPickup = drive.actionBuilder(beginPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(2,16), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(15,30), Math.toRadians(90))
                .lineToY(54, new TranslationalVelConstraint(70))
                .splineToConstantHeading(new Vector2d(15,50), Math.toRadians(-90), new TranslationalVelConstraint(70))
                .splineToConstantHeading(new Vector2d(-14,20), Math.toRadians(90), new TranslationalVelConstraint(70));

        TrajectoryActionBuilder trajectoryFrontPickUp = trajectoryShootBallsandPickup.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(-11, 32), Math.toRadians(90))
                .lineToY(49, new TranslationalVelConstraint(40))
                .splineToConstantHeading(new Vector2d(-11,30), Math.toRadians(-90), new TranslationalVelConstraint(40))
                .splineToConstantHeading(new Vector2d(-12,20), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryGateBackUp = trajectoryFrontPickUp.endTrajectory().fresh()
//                .splineToConstantHeading(new Vector2d(-0, -28), Math.toRadians(0), new TranslationalVelConstraint(70))
//                .splineToConstantHeading(new Vector2d(7,-55), Math.toRadians(-90), new TranslationalVelConstraint(40))
                .splineToLinearHeading(new Pose2d(15, 67, Math.toRadians(124)), Math.toRadians(90), new TranslationalVelConstraint(50));

        TrajectoryActionBuilder trajectoryShootBalls3 = trajectoryGateBackUp.endTrajectory().fresh()
//                .splineToConstantHeading(new Vector2d(10,-50), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-10, 20, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(70));
//                .splineToConstantHeading(new Vector2d(-12,-20), Math.toRadians(-90), new TranslationalVelConstraint(70));
//                .strafeToLinearHeading(new Vector2d(-6,-20), Math.toRadians(-90), new TranslationalVelConstraint(70));

        TrajectoryActionBuilder trajectoryGateBackUp2 = trajectoryShootBalls3.endTrajectory().fresh()
//                .splineToConstantHeading(new Vector2d(-0, -28), Math.toRadians(0), new TranslationalVelConstraint(70))
//                .splineToConstantHeading(new Vector2d(7,-55), Math.toRadians(-90), new TranslationalVelConstraint(40))
                .splineToLinearHeading(new Pose2d(15, 67, Math.toRadians(124)), Math.toRadians(90), new TranslationalVelConstraint(50));

        TrajectoryActionBuilder trajectoryShootBalls4 = trajectoryGateBackUp2.endTrajectory().fresh()
//                .splineToConstantHeading(new Vector2d(10,-50), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-10, 20, Math.toRadians(90)), Math.toRadians(90), new TranslationalVelConstraint(70));
//                .splineToConstantHeading(new Vector2d(-12,-20), Math.toRadians(-90), new TranslationalVelConstraint(70));
//                .strafeToLinearHeading(new Vector2d(-6,-20), Math.toRadians(-90), new TranslationalVelConstraint(70));

        TrajectoryActionBuilder trajectoryBackPickUp = trajectoryShootBalls4.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(33, 30), Math.toRadians(90), new TranslationalVelConstraint(30))
                .lineToY(58, new TranslationalVelConstraint(40))
                .splineToConstantHeading(new Vector2d(33,46), Math.toRadians(-90), new TranslationalVelConstraint(70))
                .splineToConstantHeading(Constants.FinalAutoTrajectories.robotEndPosRed_B, Math.toRadians(180), new TranslationalVelConstraint(70));

        TrajectoryActionBuilder trajectoryPark = trajectoryBackPickUp.endTrajectory().fresh()
                .strafeToConstantHeading(Constants.FinalAutoTrajectories.robotEndPosRed_B);
                //splineToConstantHeading(Constants.FinalAutoTrajectories.robotEndPosBlue_B, Math.toRadians(90));


        TrajectoryShootBallsAndPrep = trajectoryShootBallsandPickup.build();
        TrajectoryShootBalls3 = trajectoryShootBalls3.build();
        TrajectoryGateBackUp = trajectoryGateBackUp.build();
        TrajectoryBackPickUp = trajectoryBackPickUp.build();
        TrajectoryFrontPickUp = trajectoryFrontPickUp.build();
        TrajectoryGateBackUp2 = trajectoryGateBackUp2.build();
        TrajectoryShootBalls4 = trajectoryShootBalls4.build();
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
                               Shooter_Subsystem_Action shooterSubsystem,
                               Turret_Analog_Input_Action turretAnalog,
                               Odom_Storage_Action odomStorage,
                               Shooter_Hood_Action shooterHood){

        return
        new ParallelAction(
                    new SequentialAction(
                    scoringGate.OpenGate(),
                    scoringShooter.ShootBalls(),
                    new SleepAction(0.55),
                    new ParallelAction(
                        new SequentialAction(
                    // score preload then pickup first set of artifacts
                            new ParallelAction(
                                TrajectoryShootBallsAndPrep,
                                new SequentialAction(
                                    new SleepAction(0.75),
                                    SHOOT_REAR_MID_FRONT(intakeSubsystem),
                                    new SleepAction(0.85),
                                    scoringGate.CloseGate(),
                                    scoringShooter.ShootBalls(),
                                    INTAKE(intakeSubsystem, distanceSensor)
                                )
                            ),
                            // shoot
                            scoringGate.OpenGate(),
                            new SleepAction(0.2),
                            SHOOT_REAR_MID_FRONT(intakeSubsystem),
                            new SleepAction(0.6),
                            scoringGate.CloseGate(),
                            new ParallelAction(
                                TrajectoryFrontPickUp,
                                scoringShooter.CloseShooter2(),
                                FAST_INTAKE(intakeSubsystem, distanceSensor)
                            ),
                            scoringGate.OpenGate(),
                            new SleepAction(0.2),
                            new ParallelAction(
                                    TrajectoryGateBackUp,
                                    new SequentialAction(
                                        SHOOT_REAR_MID_FRONT(intakeSubsystem),
                                        new SleepAction(0.5),
                                        // pickup 3rd set
                                        scoringGate.CloseGate(),
                                        new SleepAction(0.5),
                                        FAST_INTAKE(intakeSubsystem, distanceSensor)
                                    )
                            ),
                            new ParallelAction(
                                    TrajectoryShootBalls3,
                                    scoringShooter.CloseShooter2()
                            ),
                            scoringGate.OpenGate(),
                            new SleepAction(0.2),
                            new ParallelAction(
                                    TrajectoryGateBackUp2,
                                    new SequentialAction(
                                            SHOOT_REAR_MID_FRONT(intakeSubsystem),
                                            new SleepAction(0.5),
                                            // pickup 2nd set
                                            scoringGate.CloseGate(),
                                            new SleepAction(0.5),
                                            FAST_INTAKE(intakeSubsystem, distanceSensor)
                                    )
                            ),
                            new ParallelAction(
                                    TrajectoryShootBalls4,
                                    scoringShooter.CloseShooter2()
                            ),
                            scoringGate.OpenGate(),
                            new SleepAction(0.2),

                            new ParallelAction(
                                TrajectoryBackPickUp,
                                new SequentialAction(
                                        SHOOT_REAR_MID_FRONT(intakeSubsystem),
                                        new SleepAction(0.5),
                                        // pickup 2nd set
                                        scoringGate.CloseGate(),
                                        new SleepAction(0.4),
                                        SLOW_INTAKE(intakeSubsystem, distanceSensor)
                                ),
                                scoringShooter.CloseShooter2()
                            ),
                            scoringGate.OpenGate(),
                            new SleepAction(0.35),
                                SHOOT_REAR_MID_FRONT(intakeSubsystem),
                            new SleepAction(0.75)
                        ),
                        turretAnalog.GetTotalTurretAngle()
                    )
                ),
                shooterSubsystem.AutoAim(),
                odomStorage.sendOdomCoords(),
                shooterHood.AutoHood()
        );
    }

    public Action SHOOT_REAR_MID_FRONT(Intake_Subsystem_Action intakeSubsystem){
        return new SequentialAction(
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FORWARD_FORWARD_REVERSE)
        );
    }

    public Action READTURRETPOSITION(Shooter_Subsystem_Action shooterSubsystem){
        return new SequentialAction(
                shooterSubsystem.ReadTurretPosition()
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
