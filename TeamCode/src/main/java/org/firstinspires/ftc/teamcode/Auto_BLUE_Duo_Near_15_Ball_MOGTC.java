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
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Shooter_Subsystem_Action;
import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;

@Configurable
@Autonomous (name = "BLUE Duo Near 15 MOGTC", group = "Blue Alliance")
public class Auto_BLUE_Duo_Near_15_Ball_MOGTC extends LinearOpMode {

    private MecanumDrive drive;
    Action TrajectoryRoute1,
            TrajectoryRoute2,
            TrajectoryRoute3,
            TrajectoryRoute4,
            TrajectoryRoute5,
            TrajectoryRoute6;

    public void runOpMode() throws InterruptedException{
        Pose2d beginPose = new Pose2d(-40, -57, Math.toRadians(-90));
        drive = new MecanumDrive(hardwareMap, beginPose);
        Scoring_Shooter_Action scoringShooter = new Scoring_Shooter_Action(hardwareMap);
        Intake_Subsystem_Action intakeSubsystem = new Intake_Subsystem_Action(hardwareMap);
        Distance_Sensor_Action distanceSensor = new Distance_Sensor_Action(hardwareMap);
        Scoring_Gate_Action scoringGate = new Scoring_Gate_Action(hardwareMap);
        Shooter_Subsystem_Action shooterSubsystem = new Shooter_Subsystem_Action(hardwareMap, Shooter_Subsystem_Action.Pipeline.BLUE);
        Turret_Analog_Input_Action turretAnalog = new Turret_Analog_Input_Action(hardwareMap);
        Odom_Storage_Action odomStorage = new Odom_Storage_Action(hardwareMap, drive);


        shooterSubsystem.setPower(0);
        buildTrajectories(drive, beginPose);
        waitForStart();
        PostStorage.currentPose = drive.localizer.getPose();
        Actions.runBlocking(
                new SequentialAction(
                        scorePreload(scoringShooter, intakeSubsystem, distanceSensor, scoringGate, shooterSubsystem, turretAnalog, odomStorage)
                )
        );
        PostStorage.currentPose = drive.localizer.getPose();
    }

    private void buildTrajectories(MecanumDrive drive, Pose2d beginPose){
        // preload and pickup mid
        TrajectoryActionBuilder trajectoryRoute1 = drive.actionBuilder(beginPose) // preload and pickup MID and open gate
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(2,-16), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(15,-30), Math.toRadians(-90))
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(13,-58), Math.toRadians(-90), new TranslationalVelConstraint(40))
                .splineToConstantHeading(new Vector2d(2,-46), Math.toRadians(-90), new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(-4,-60), Math.toRadians(-90), new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(-6,-50), Math.toRadians(90), new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(-12,-20), Math.toRadians(-90), new TranslationalVelConstraint(70));

        TrajectoryActionBuilder trajectoryRoute2= trajectoryRoute1.endTrajectory().fresh() //Gate Open
                .splineToConstantHeading(new Vector2d(-0, -28), Math.toRadians(0), new TranslationalVelConstraint(70))
                .splineToConstantHeading(new Vector2d(7,-55), Math.toRadians(-90), new TranslationalVelConstraint(40))
                .splineToLinearHeading(new Pose2d(20, -64, Math.toRadians(-120)), Math.toRadians(-90), new TranslationalVelConstraint(70));

        TrajectoryActionBuilder trajectoryRoute3 = trajectoryRoute2.endTrajectory().fresh() // Shoot Gate Balls
                .strafeToLinearHeading(new Vector2d(-6,-20), Math.toRadians(-90), new TranslationalVelConstraint(70));

        TrajectoryActionBuilder trajectoryRoute4 = trajectoryRoute3.endTrajectory().fresh() // pickup TUNNEL
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(28, -30), Math.toRadians(-90), new TranslationalVelConstraint(30))
                .setReversed(false)
                .lineToY(-62, new TranslationalVelConstraint(40))
                .splineToConstantHeading(new Vector2d(26,-48), Math.toRadians(90), new TranslationalVelConstraint(70))
                .splineToConstantHeading(new Vector2d(24,-62), Math.toRadians(-90), new TranslationalVelConstraint(30))
                .splineToConstantHeading(new Vector2d(0, -22), Math.toRadians(-180), new TranslationalVelConstraint(70))
                .splineToConstantHeading(new Vector2d(-12, -20), Math.toRadians(-90), new TranslationalVelConstraint(70));

        TrajectoryActionBuilder trajectoryRoute5 = trajectoryRoute4.endTrajectory().fresh() // pickup CLOSE
                .splineToConstantHeading(new Vector2d(-11, -32), Math.toRadians(-90))
                .lineToY(-50, new TranslationalVelConstraint(40))
                .splineToConstantHeading(new Vector2d(-11,-30), Math.toRadians(90), new TranslationalVelConstraint(40))
                .splineToConstantHeading(new Vector2d(-10,-20), Math.toRadians(-90));

        TrajectoryActionBuilder trajectoryRoute6 = trajectoryRoute5.endTrajectory().fresh()
                .strafeToConstantHeading(Constants.FinalAutoTrajectories.robotEndPosBlue_B);

        TrajectoryRoute1 = trajectoryRoute1.build();
        TrajectoryRoute2 = trajectoryRoute2.build();
        TrajectoryRoute3 = trajectoryRoute3.build();
        TrajectoryRoute4 = trajectoryRoute4.build();
        TrajectoryRoute5 = trajectoryRoute5.build();
        TrajectoryRoute6 = trajectoryRoute6.build();
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
                               Odom_Storage_Action odomStorage){

        return
        new ParallelAction(
                    new SequentialAction(
                    scoringGate.OpenGate(),
                    scoringShooter.ShootBalls(),
                    new SleepAction(0.5),
                    new ParallelAction(
                        new SequentialAction(
                    // score preload then pickup first set of artifacts
                            new ParallelAction(
                                TrajectoryRoute1,
                                SHOOT_PRELOAD(intakeSubsystem, scoringGate, scoringShooter, distanceSensor)
                            ),
                            SHOOT_CLOSE(intakeSubsystem, scoringGate),
                            new ParallelAction(
                                    TrajectoryRoute2,
                                    new SequentialAction(
                                            new SleepAction(0.5),
                                            FAST_INTAKE(intakeSubsystem, distanceSensor)
                                    )
                            ),
                            new ParallelAction(
                                    TrajectoryRoute3,
                                    scoringShooter.CloseShooter2()
                            ),
                            // shoot
                            SHOOT_CLOSE(intakeSubsystem, scoringGate),
                            new ParallelAction(
                                TrajectoryRoute4,
                                    new SequentialAction(
                                            new SleepAction(0.5),
                                            FAST_INTAKE(intakeSubsystem, distanceSensor)
                                    )
                            ),
                            SHOOT_CLOSE(intakeSubsystem, scoringGate),
                            new ParallelAction(
                                    TrajectoryRoute5,
                                    new SequentialAction(
                                            new SleepAction(0.5),
                                            FAST_INTAKE(intakeSubsystem, distanceSensor)
                                    )
                            ),
                            SHOOT_CLOSE(intakeSubsystem, scoringGate),
                            TrajectoryRoute6
                        ),
                        turretAnalog.GetTotalTurretAngle()
                    )
//                    TrajectoryRoute4,
//                    TrajectoryRoute6
                ),
                shooterSubsystem.AutoAim(),
                odomStorage.sendOdomCoords()
        );
    }

    public Action SHOOT_PRELOAD(Intake_Subsystem_Action intakeSubsystem, Scoring_Gate_Action scoringGate, Scoring_Shooter_Action scoringShooter, Distance_Sensor_Action distanceSensor){
        return new SequentialAction(
                new SleepAction(0.75),
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FORWARD_FORWARD_REVERSE),
                new SleepAction(0.75),
                scoringGate.CloseGate(),
                scoringShooter.CloseShooter2(),
                INTAKE(intakeSubsystem, distanceSensor)
        );
    }

    public Action SHOOT_CLOSE(Intake_Subsystem_Action intakeSubsystem, Scoring_Gate_Action scoringGate){
        return new SequentialAction(
                scoringGate.OpenGate(),
                new SleepAction(0.35),
                intakeSubsystem.action(Intake_Subsystem_Action.IntakeMode.FORWARD_FORWARD_REVERSE),
                new SleepAction(0.5),
                scoringGate.CloseGate()
        );
    }

    public Action SHOOT_FAR(Intake_Subsystem_Action intakeSubsystem){
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
