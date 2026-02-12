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

import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Actions.Distance_Sensor_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Actions.Intake_Subsystem_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Gate_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Shooter_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Shooter_Subsystem_Action;
import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;

@Configurable
@Autonomous (name = "RED Duo Far", group = "Red Alliance")
public class Auto_RED_Duo_Far extends LinearOpMode {

    private MecanumDrive drive;
    Action
            TrajectoryFarSpikePickUp,
            TrajectoryHumanPickUp1,
            TrajectoryHumanPickup2,
            TrajectoryHumanPickup3,
            TrajectoryHumanPickup4,
            TrajectoryPark;

    public void runOpMode() throws InterruptedException{
        Pose2d beginPose = new Pose2d(62, 12, Math.toRadians(90));
        drive = new MecanumDrive(hardwareMap, beginPose);
        Scoring_Shooter_Action scoringShooter = new Scoring_Shooter_Action(hardwareMap);
        Intake_Subsystem_Action intakeSubsystem = new Intake_Subsystem_Action(hardwareMap);
        Distance_Sensor_Action distanceSensor = new Distance_Sensor_Action(hardwareMap);
        Scoring_Gate_Action scoringGate = new Scoring_Gate_Action(hardwareMap);
        Shooter_Subsystem_Action shooterSubsystem = new Shooter_Subsystem_Action(hardwareMap, Shooter_Subsystem_Action.Pipeline.RED);

        buildTrajectories(drive, beginPose);
        waitForStart();
        Actions.runBlocking(
            new SequentialAction(
                scorePreload(scoringShooter, intakeSubsystem, distanceSensor, scoringGate, shooterSubsystem)
            )
        );
    }

    private void buildTrajectories(MecanumDrive drive, Pose2d beginPose){

        TrajectoryActionBuilder trajectoryFarSpikePickUp = drive.actionBuilder(beginPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(36, 30), Math.toRadians(90))
                .lineToY(58, new TranslationalVelConstraint(70))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(50, 18), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryHumanPickUp1 = trajectoryFarSpikePickUp.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(68,66), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(66,52), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(68,70), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(50,18), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryHumanPickup2 = trajectoryHumanPickUp1.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(68,66), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(66,52), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(68,70), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(50,18), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryHumanPickup3 = trajectoryHumanPickup2.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(68,67), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(66,52), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(68,70), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(50,18), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryHumanPickup4 = trajectoryHumanPickup3.endTrajectory().fresh()
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(68,66), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(62,52), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(68,70), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(50,18), Math.toRadians(90));

        TrajectoryActionBuilder trajectoryPark = trajectoryHumanPickup4.endTrajectory().fresh()
                .splineToConstantHeading(new Vector2d(36, 48), Math.toRadians(-90));

        TrajectoryHumanPickup4 = trajectoryHumanPickup4.build();
        TrajectoryHumanPickup3 = trajectoryHumanPickup3.build();
        TrajectoryHumanPickup2 = trajectoryHumanPickup2.build();
        TrajectoryHumanPickUp1 = trajectoryHumanPickUp1.build();
        TrajectoryFarSpikePickUp = trajectoryFarSpikePickUp.build();
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
                        // shoot preload
                        scoringGate.OpenGate(),
                        scoringShooter.FartherShooter(),
                        new SleepAction(2.5),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(0.75),
                        scoringGate.CloseGate(),
                        scoringShooter.FarShooter(),

                        // pickup furthest spike mark and go to shooting zone
                        new ParallelAction(
                            TrajectoryFarSpikePickUp,
                            SLOW_INTAKE(intakeSubsystem, distanceSensor)
                        ),

                        // shoot
                        new SleepAction(0.1),
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(0.65),
                        scoringGate.CloseGate(),

                        // pickup from human player then go to shooting zone
                        new ParallelAction(
                                TrajectoryHumanPickUp1,
                                new SequentialAction(
                                    new SleepAction(0.35),
                                    INTAKE(intakeSubsystem, distanceSensor)
                                )
                        ),

                        // shoot
                        new SleepAction(0.05),
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(0.65),
                        scoringGate.CloseGate(),

                        // pickup from human player then go to shooting zone
                        new ParallelAction(
                                TrajectoryHumanPickup2,
                                new SequentialAction(
                                        new SleepAction(0.35),
                                        INTAKE(intakeSubsystem, distanceSensor)
                                )
                        ),

                        // shoot
                        new SleepAction(0.05),
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(0.65),
                        scoringGate.CloseGate(),

                        // pickup from human player then go to shooting zone
                        new ParallelAction(
                                TrajectoryHumanPickup3,
                                new SequentialAction(
                                        new SleepAction(0.35),
                                        INTAKE(intakeSubsystem, distanceSensor)
                                )
                        ),

                        // shoot
                        new SleepAction(0.05),
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(0.5),
                        scoringGate.CloseGate(),

                        // pickup from human player then go to shooting zone
                        new ParallelAction(
                                TrajectoryHumanPickup4,
                                new SequentialAction(
                                        new SleepAction(0.35),
                                        INTAKE(intakeSubsystem, distanceSensor)
                                )
                        ),

                        // shoot
                        new SleepAction(0.05),
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(0.5),
                        scoringGate.CloseGate(),

                        TrajectoryPark
                ),

                shooterSubsystem.AutoAim() // enable auto aim for the entire auto
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
