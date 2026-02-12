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
@Autonomous (name = "Blue Duo Auto", group = "Duo Auto")
public class Duo_Far_Auto_Blue extends LinearOpMode {

    private MecanumDrive drive;
    Action
            TrajectoryGatePickUp,
            TrajectoryGateBackUp,
            TrajectoryShootBalls3,
            TrajectoryBackPickUp;




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

        TrajectoryActionBuilder trajectoryBackPickUp = drive.actionBuilder(beginPose)
                .splineToConstantHeading(new Vector2d(36, -30), Math.toRadians(-90))
                .lineToY(-67, new TranslationalVelConstraint(70))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(50, -18), Math.toRadians(90), new TranslationalVelConstraint(60));

        TrajectoryActionBuilder trajectoryGatePickUp = trajectoryBackPickUp.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(66,-66, Math.toRadians(0)), Math.toRadians(-90), new TranslationalVelConstraint(70))
                .lineToX(24);

        TrajectoryActionBuilder trajectoryShootBalls3 = trajectoryGatePickUp.endTrajectory().fresh()
                .setReversed(false)
                .splineToLinearHeading(new Pose2d(50,-18, Math.toRadians(-90)), Math.toRadians(-90), new TranslationalVelConstraint(70));
        TrajectoryShootBalls3 = trajectoryShootBalls3.build();
        TrajectoryGatePickUp = trajectoryGatePickUp.build();
        TrajectoryBackPickUp = trajectoryBackPickUp.build();
    }

    public Action spinShooter(Scoring_Shooter_Action scoringShooter){
        return new SequentialAction(
                scoringShooter.ShootBalls()
        );
    }
    public Action scorePreload(Scoring_Shooter_Action scoringShooter,
                               Intake_Subsystem_Action intakeSubsystem,
                                Distance_Sensor_Action distanceSensor,
                                Scoring_Gate_Action scoringGate ,
                               Shooter_Subsystem_Action shooterSubsystem){

        return
        new ParallelAction(
            new SequentialAction(
                    scoringGate.OpenGate(),
                    //new SleepAction(0.15),
                    scoringShooter.FarShooter(),
                    new SleepAction(2.5),
                    SHOOT_REAR_MID_FRONT(intakeSubsystem),
                    new SleepAction(0.5),
                    new ParallelAction(
                            TrajectoryBackPickUp,
                            SLOW_INTAKE(intakeSubsystem, distanceSensor),
                            scoringShooter.FarShooter()
                    ),
                    scoringGate.OpenGate(),
                    new SleepAction(0.35),
                    FAR_SHOOT(intakeSubsystem),
                    new SleepAction(0.75),
                    new ParallelAction(
                        TrajectoryGateBackUp,
                        scoringShooter.FarShooter(),
                        new SequentialAction(
                                new SleepAction(0.35),
                                INTAKE(intakeSubsystem, distanceSensor)
                        )
                    ),
                    new ParallelAction(
                            TrajectoryShootBalls3,
                            scoringShooter.FarShooter()
                    ),
                    scoringGate.OpenGate(),
                    new SleepAction(0.35),
                    FAR_SHOOT(intakeSubsystem),
                    new SleepAction(0.75)
            ),
            shooterSubsystem.AutoAim()
                // score preload then pickup first set of artifacts
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
    public Action GATEINTAKE(Intake_Subsystem_Action intakeSubsystem, Distance_Sensor_Action distanceSensor){
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
