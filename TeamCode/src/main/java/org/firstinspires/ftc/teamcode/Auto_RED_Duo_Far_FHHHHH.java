package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Actions.Distance_Sensor_Action;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Actions.Odom_Storage_Action;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Actions.Turret_Analog_Input_Action;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Actions.Intake_Subsystem_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Gate_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Scoring_Shooter_Action;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions.Shooter_Subsystem_Action;
import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;

@Configurable
@Autonomous (name = "RED Duo Far F5H", group = "Red Alliance")
public class Auto_RED_Duo_Far_FHHHHH extends LinearOpMode {

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
        Turret_Analog_Input_Action turretAnalog = new Turret_Analog_Input_Action(hardwareMap);
        Odom_Storage_Action odomStorage = new Odom_Storage_Action(hardwareMap, drive);

        buildTrajectories(drive, beginPose);

        while (!isStarted()){
            Actions.runBlocking(
                    new SequentialAction(READTURRETPOSITION(shooterSubsystem)
                    )
            );
        }

        waitForStart();
        Actions.runBlocking(
            new SequentialAction(
                scorePreload(scoringShooter, intakeSubsystem, distanceSensor, scoringGate, shooterSubsystem, turretAnalog, odomStorage)
            )
        );
    }

    private void buildTrajectories(MecanumDrive drive, Pose2d beginPose){

        TrajectoryActionBuilder trajectoryFarSpikePickUp = drive.actionBuilder(beginPose) // FAR
                .splineToConstantHeading(new Vector2d(36, 30), Math.toRadians(90))
                .lineToY(60)
                .splineToConstantHeading(new Vector2d(36,46), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(56, 16), Math.toRadians(0));

        TrajectoryActionBuilder trajectoryHumanPickUp1 = trajectoryFarSpikePickUp.endTrajectory().fresh() // HUMAN PLAYER
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(68,62), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(66,50), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60,64), Math.toRadians(90))
                .lineToY(16);

        TrajectoryActionBuilder trajectoryHumanPickup2 = trajectoryHumanPickUp1.endTrajectory().fresh() // HUMAN PLAYER
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(68,62), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(66,50), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60,64), Math.toRadians(90))
                .lineToY(16);

        TrajectoryActionBuilder trajectoryHumanPickup3 = trajectoryHumanPickup2.endTrajectory().fresh() // HUMAN PLAYER
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(68,62), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(66,50), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60,64), Math.toRadians(90))
                .lineToY(16);

        TrajectoryActionBuilder trajectoryHumanPickup4 = trajectoryHumanPickup3.endTrajectory().fresh() // HUMAN PLAYER
                .setReversed(false)
                .splineToConstantHeading(new Vector2d(68,62), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(62,50), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(60,64), Math.toRadians(90))
                .lineToY(16);

        TrajectoryActionBuilder trajectoryPark = trajectoryHumanPickup4.endTrajectory().fresh() // HUMAN PLAYER
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
                               Shooter_Subsystem_Action shooterSubsystem,
                               Turret_Analog_Input_Action turretAnalog,
                               Odom_Storage_Action odomStorage){

        return
        new ParallelAction(
                new SequentialAction(
                        // shoot preload
                        scoringGate.OpenGate(),
                        scoringShooter.FarShooter(),
                        new SleepAction(2.5),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(1),
                        scoringGate.CloseGate(),
                        scoringShooter.FartherShooter(),

                        // pickup furthest spike mark and go to shooting zone
                        new ParallelAction(
                            TrajectoryFarSpikePickUp,
                            SLOW_INTAKE(intakeSubsystem, distanceSensor)
                        ),

                        // shoot
//                        new SleepAction(0.1),
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(1),
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
//                        new SleepAction(0.05),
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(1),
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
//                        new SleepAction(0.05),
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(1),
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
//                        new SleepAction(0.05),
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(1),
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
//                        new SleepAction(0.05),
                        scoringGate.OpenGate(),
                        new SleepAction(0.35),
                        FAR_SHOOT(intakeSubsystem),
                        new SleepAction(1),
                        scoringGate.CloseGate(),

                        TrajectoryPark
                ),
                shooterSubsystem.AutoAim(),
                odomStorage.sendOdomCoords(),
                turretAnalog.GetTotalTurretAngle()// enable auto aim for the entire auto
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

    public Action READTURRETPOSITION(Shooter_Subsystem_Action shooterSubsystem){
        return new SequentialAction(
                shooterSubsystem.ReadTurretPosition()
        );
    }

}
