package org.firstinspires.ftc.teamcode.subsystems.robot;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.bylazar.gamepad.GamepadManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Distance_Sensor;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Light_Indicator;
import org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Commands.ActuateLightIndicatorCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.driveCommands.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.driveCommands.SlowModeCommand;
import org.firstinspires.ftc.teamcode.subsystems.drive.driveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.endgame.Endgame_Commands.MoveEndgameKickstandCommand;
import org.firstinspires.ftc.teamcode.subsystems.endgame.Endgame_Kickstand;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Indexer;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.intake.intake_commands.SpinIntakeSubsystemCommand;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Scoring_Gate;
import org.firstinspires.ftc.teamcode.subsystems.scoring.Shooter_Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands.AutoAimTurretCommand;
import org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_commands.MoveScoringGateCommand;

import org.firstinspires.ftc.teamcode.subsystems.PostStorage;

import com.bylazar.gamepad.PanelsGamepad;

public class MyRobot extends Robot {
    public GamepadManager g1Manager = PanelsGamepad.INSTANCE.getFirstManager();
    public GamepadManager g2Manager = PanelsGamepad.INSTANCE.getSecondManager();
    public final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    public LinearOpMode opMode;
    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public driveSubsystem drive;
    public Intake_Subsystem intakeSubsystem;
    public Shooter_Subsystem shooterSubsystem;
    public Endgame_Kickstand endgameKickstand;
    //public Camera_Subsystem cameraSubsystem;
    public Distance_Sensor distanceSensor;
    public Light_Indicator lightIndicator;
    public Intake_Indexer intakeIndexer;
    public Scoring_Gate scoringGate;
    public GamepadEx driver;
    public GamepadEx operator;
    public DistanceUnit distanceUnit;

    public static boolean shooterSensorTriggered = false;
    public static boolean frontSensorTriggered = false;
    public static boolean rearSensorTriggered = false;
    public static boolean telemetryon = false;
    public static double targetAngle = 0;
    public double[] turretProp = {0, 0};
    public double turretBearing = 0;
    public double turretRange = 0;

    public DefaultDriveCommand defaultDriveCommand;
    public SlowModeCommand slowModeCommand;
    public AutoAimTurretCommand autoAimCommand;
    public MoveScoringGateCommand gateCommand;

    public static ElapsedTime shooterTime;
    public static boolean startShooter = true;

    public static int CLOSE_SHOOT_GATE_OPEN_DELAY = 100;
    public static int CLOSE_SHOOT_FIRST_BALL_DELAY = 25;
    public static int FAR_SHOOT_GATE_OPEN_DELAY = 100;
    public static int FAR_SHOOT_FIRST_BALL_DELAY = 25;
    public static double SHOOTING_DISTANCE_THRESHOLD = 90;

    public enum TeleOpModeType {
        Field, Robot
    }

    public enum TeleOpMode {
        BLUE,
        RED
    }

    // the constructor with a specified opmode type
    public MyRobot(LinearOpMode opMode, TeleOpModeType type, TeleOpMode mode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.driver = new GamepadEx(opMode.gamepad1);
        this.operator = new GamepadEx(opMode.gamepad2);
        var g1 = g1Manager.asCombinedFTCGamepad(opMode.gamepad1);
        var g2 = g2Manager.asCombinedFTCGamepad(opMode.gamepad2);
        initTele(mode);
        if (type == TeleOpModeType.Field) {
            drive.setFieldOriented(true);
        } else {
            drive.setFieldOriented(false);
        }
    }


    /*
     * Initialize teleop or autonomous, depending on which is used
     */
    public void initTele(TeleOpMode mode) {
//        MyRobot.disable();
//        shooterTime = new ElapsedTime();
//        drive = new driveSubsystem(hardwareMap, new Pose2d(0, 0, 0));
//        endgameKickstand = new Endgame_Kickstand(this);
//        intakeSubsystem = new Intake_Subsystem(this);
//        distanceSensor = new Distance_Sensor(this);
//        scoringGate = new Scoring_Gate(this);
//        shooterSubsystem = new Shooter_Subsystem(this, Shooter_Subsystem.Team.RED);
//        lightIndicator = new Light_Indicator(this);
//        MyRobot.enable();

        if (mode == TeleOpMode.RED){
            targetAngle = 1.5;
            shooterTime = new ElapsedTime();
            drive = new driveSubsystem(hardwareMap, new Pose2d(0, 0, 0));
            endgameKickstand = new Endgame_Kickstand(this);
            intakeSubsystem = new Intake_Subsystem(this);
            distanceSensor = new Distance_Sensor(this);
            scoringGate = new Scoring_Gate(this);
            shooterSubsystem = new Shooter_Subsystem(this, Shooter_Subsystem.Team.RED);
            lightIndicator = new Light_Indicator(this);
            register(drive, intakeSubsystem, lightIndicator, distanceSensor, scoringGate, shooterSubsystem);

            defaultDriveCommand = new DefaultDriveCommand(drive,
                    driver::getLeftX,
                    driver::getLeftY,
                    driver::getRightX

            );

            slowModeCommand = new SlowModeCommand(drive,
                    driver::getLeftX,
                    driver::getLeftY,
                    driver::getRightX
            );

            autoAimCommand = new AutoAimTurretCommand(
                    shooterSubsystem,
                     targetAngle,
                    this,
                    startShooter,
                    operator::getRightX,
                    AutoAimTurretCommand.Team.Red
            );

            CommandScheduler.getInstance().run();
            CommandScheduler.getInstance().setDefaultCommand(drive, defaultDriveCommand);
            CommandScheduler.getInstance().setDefaultCommand(shooterSubsystem, autoAimCommand);

            Button driverIntakeFront = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
            Button driveSpeedButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
            Button driveTelemetry = new GamepadButton(driver, GamepadKeys.Button.BACK);
            Button drivesSoot = new GamepadButton(driver, GamepadKeys.Button.X);
            Button endgameActivate =  new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
            Button endgameDeActivate = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);

            endgameKickstand.setState(Endgame_Kickstand.EndgameState.UP);

            endgameActivate.whenPressed(new MoveEndgameKickstandCommand(endgameKickstand, Endgame_Kickstand.EndgameState.DOWN));

            endgameDeActivate.whenPressed(new MoveEndgameKickstandCommand(endgameKickstand, Endgame_Kickstand.EndgameState.UP));

            driveSpeedButton
                    .whenHeld(slowModeCommand)
                    .whenReleased(defaultDriveCommand);

            driveTelemetry.whenPressed(
                    new ConditionalCommand(
                            new ParallelCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetryon = false)
                            ),
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clear()),
                                    new InstantCommand(()-> telemetry.addLine("Reminder (Stand Behind The Robot): \n")),
                                    new InstantCommand(()-> telemetry.addLine("Left on Left Joystick = Outwards On Right, Inwards on Left \n")),
                                    new InstantCommand(()-> telemetry.addLine("Right on Left Joystick = Inwards On Right, Outwards on Left \n")),
                                    new InstantCommand(()-> telemetry.addLine("Left on Right Joystick = Forward On Right, Backward on Left \n")),
                                    new InstantCommand(()-> telemetry.addLine("Right on Right Joystick = Backward On Right, Forward on Left \n")),
                                    new InstantCommand(()-> telemetry.update()),
                                    new InstantCommand(()-> telemetryon = true)
                            ),
                            () -> {return (telemetryon);}

                    )
            );

            Button intakeFront = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
            Button intakeBack = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);
            Button intakeShoot1 = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
            Button shooterFar = new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER);
            Button shooterStart = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
            Button shooterStartClose = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);
            Button shooterStartCloseClose = new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT);
            Button shooterStop = new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT);
            Button sensorTele = new GamepadButton(operator, GamepadKeys.Button.BACK);
            Button directIntakeShoot = new GamepadButton(operator, GamepadKeys.Button.X);

            scoringGate.setState(Scoring_Gate.ScoringGState.CLOSE);

            intakeFront.whileHeld(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.addLine("Intake Artifacts From Front")),
                                    new InstantCommand(()-> telemetry.addData("Shooter Sensor Triggered", shooterSensorTriggered)),
                                    new InstantCommand(()-> telemetry.addData("Back Sensor Triggered", rearSensorTriggered)),
                                    new InstantCommand(()-> telemetry.addData("Front Sensor Triggered", frontSensorTriggered)),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(

                                    new InstantCommand(()-> {
                                        if (shooterSensorTriggered == false) {
                                            shooterSensorTriggered = distanceSensor.shootSensorTriggered();
                                        }
                                        if (rearSensorTriggered == false) {
                                            rearSensorTriggered = distanceSensor.rearSensorTriggered() && shooterSensorTriggered;
                                        }
                                        if (frontSensorTriggered == false) {
                                            frontSensorTriggered = distanceSensor.frontSensorTriggered() && rearSensorTriggered && shooterSensorTriggered;
                                        }
                                    }),
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.CLOSE),
                                    new ConditionalCommand(
                                            new ParallelCommandGroup(
                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                                                    new ActuateLightIndicatorCommand(lightIndicator, Light_Indicator.LightIndicatorState.BLUE)
                                            ),
                                            new ConditionalCommand(
                                                    new ParallelCommandGroup(
                                                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.FORWARD_OFF_OFF)
                                                    ),
                                                    new ConditionalCommand(
                                                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.FORWARD_SLOWFORWARD_OFF),
                                                            new ParallelCommandGroup(
                                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.FORWARD_FORWARD_REVERSE),
                                                                    new ActuateLightIndicatorCommand(lightIndicator, Light_Indicator.LightIndicatorState.YELLOW)
                                                            ),
                                                            () -> {return shooterSensorTriggered;}
                                                    ),
                                                    () -> {return (rearSensorTriggered);}
                                            ),
                                            () -> {return (frontSensorTriggered);}
                                    )
                            )
                    )
            ).whenReleased(
                    new SequentialCommandGroup(
                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                            new InstantCommand(()-> {
                                shooterSensorTriggered = false;
                                frontSensorTriggered = false;
                                rearSensorTriggered = false;
                            })
                    )
            );

            driverIntakeFront.whileHeld(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.addLine("Intake Artifacts From Front")),
                                    new InstantCommand(()-> telemetry.addData("Shooter Sensor Triggered", shooterSensorTriggered)),
                                    new InstantCommand(()-> telemetry.addData("Back Sensor Triggered", rearSensorTriggered)),
                                    new InstantCommand(()-> telemetry.addData("Front Sensor Triggered", frontSensorTriggered)),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(

                                    new InstantCommand(()-> {
                                        if (shooterSensorTriggered == false) {
                                            shooterSensorTriggered = distanceSensor.shootSensorTriggered();
                                        }
                                        if (rearSensorTriggered == false) {
                                            rearSensorTriggered = distanceSensor.rearSensorTriggered() && shooterSensorTriggered;
                                        }
                                        if (frontSensorTriggered == false) {
                                            frontSensorTriggered = distanceSensor.frontSensorTriggered() && rearSensorTriggered && shooterSensorTriggered;
                                        }
                                    }),
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.CLOSE),
                                    new ConditionalCommand(
                                            new ParallelCommandGroup(
                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                                                    new ActuateLightIndicatorCommand(lightIndicator, Light_Indicator.LightIndicatorState.BLUE)
                                            ),
                                            new ConditionalCommand(
                                                    new ParallelCommandGroup(
                                                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.FORWARD_OFF_OFF)
                                                    ),
                                                    new ConditionalCommand(
                                                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.FORWARD_SLOWFORWARD_OFF),
                                                            new ParallelCommandGroup(
                                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.FORWARD_FORWARD_REVERSE),
                                                                    new ActuateLightIndicatorCommand(lightIndicator, Light_Indicator.LightIndicatorState.YELLOW)
                                                            ),
                                                            () -> {return shooterSensorTriggered;}
                                                    ),
                                                    () -> {return (rearSensorTriggered);}
                                            ),
                                            () -> {return (frontSensorTriggered);}
                                    )
                            )
                    )
            ).whenReleased(
                    new SequentialCommandGroup(
                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                            new InstantCommand(()-> {
                                shooterSensorTriggered = false;
                                frontSensorTriggered = false;
                                rearSensorTriggered = false;
                            })
                    )
            );

            drivesSoot.whenPressed(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetry.addLine("Driver Shooting")),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> {
                                        turretProp = shooterSubsystem.detectAprilTag();
                                        turretBearing = (turretProp[0]);
                                        turretRange = (turretProp[1]);

                                        if (Math.abs(turretBearing) != 0) {
                                            if (turretRange > SHOOTING_DISTANCE_THRESHOLD) {
                                                new SequentialCommandGroup(
                                                        new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.OPEN),
                                                        new WaitCommand(FAR_SHOOT_GATE_OPEN_DELAY),
                                                        new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.HALFSHOOT_SHOOT_SHOOT),
                                                        new WaitCommand(FAR_SHOOT_FIRST_BALL_DELAY),
                                                        new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.HALFSHOOT_SHOOT_SHOOT),
                                                        new WaitCommand(2000),
                                                        new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT)
                                                );
                                            }
                                        } else{
                                            new SequentialCommandGroup(
                                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.OPEN),
                                                    new WaitCommand(CLOSE_SHOOT_GATE_OPEN_DELAY),
                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.OFF_SLOWFORWARD_REVERSE),
                                                    new WaitCommand(CLOSE_SHOOT_FIRST_BALL_DELAY),
                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.THREE_QUARTERS_SHOOT),
                                                    new WaitCommand(2000),
                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT)
                                            );
                                        }
                                    }
                                    )
                            )
                    )
            );

            intakeBack.whileHeld(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetry.addLine("Intake Artifacts From Back")),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> {
                                        if (frontSensorTriggered == false){
                                            frontSensorTriggered = distanceSensor.frontSensorTriggered();
                                        }
                                        if (shooterSensorTriggered == false) {
                                            shooterSensorTriggered = distanceSensor.shootSensorTriggered();
                                        }
                                        if (rearSensorTriggered == false) {
                                            rearSensorTriggered = distanceSensor.rearSensorTriggered() && shooterSensorTriggered;
                                        }
                                    }),
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.CLOSE),
                                    new ConditionalCommand(
                                            new ParallelCommandGroup(
                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                                                    new ActuateLightIndicatorCommand(lightIndicator, Light_Indicator.LightIndicatorState.BLUE)
                                            ),
                                            new ConditionalCommand(
                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.OFF_OFF_SLOWREVERSE),
                                                    new ConditionalCommand(
                                                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.OFF_OFF_REVERSE),
                                                            new ParallelCommandGroup(
                                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.OFF_REVERSE_REVERSE),
                                                                    new ActuateLightIndicatorCommand(lightIndicator, Light_Indicator.LightIndicatorState.YELLOW)
                                                            ),
                                                            () -> {return frontSensorTriggered;}
                                                    ),
                                                    ()-> {return shooterSensorTriggered;}
                                            ),
                                            () -> {return (rearSensorTriggered);}
                                    )
                            )
                    )

            ).whenReleased(
                    new SequentialCommandGroup(
                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                            new InstantCommand(()-> {
                                shooterSensorTriggered = false;
                                frontSensorTriggered = false;
                                rearSensorTriggered = false;
                            })

                    )
            );

            intakeShoot1.whenPressed(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetry.addLine("Close Range Shooting")),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.OPEN),
                                    new WaitCommand(CLOSE_SHOOT_GATE_OPEN_DELAY),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.OFF_SLOWFORWARD_REVERSE),
                                    new WaitCommand(CLOSE_SHOOT_FIRST_BALL_DELAY),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.CLOSESHOOT_SHOOT_SHOOT),
                                    new WaitCommand(2000),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT)
                            )
                    )
            );

            directIntakeShoot.whileHeld(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetry.addLine("Shoot while Intaking")),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.OPEN),
                                    new WaitCommand(250),
                                    new ParallelCommandGroup(
                                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.CLOSESHOOT_SHOOT_SHOOT)
                                    )
                            )
                    )
            ).whenReleased(
                    new SequentialCommandGroup(
                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                            new InstantCommand(()-> {
                                shooterSensorTriggered = false;
                                frontSensorTriggered = false;
                                rearSensorTriggered = false;
                            })

                    )
            );

            shooterFar.whenPressed(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetry.addLine("Far Range Shooting")),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.OPEN),
                                    new WaitCommand(FAR_SHOOT_GATE_OPEN_DELAY),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.OFF_SLOWFORWARD_SLOWREVERSE),
                                    new WaitCommand(FAR_SHOOT_FIRST_BALL_DELAY),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.HALFSHOOT_SHOOT_SHOOT),
                                    new WaitCommand(2000),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT)
                            )
                    )
            );

            sensorTele.whileHeld(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> telemetry.clearAll()),
                            new InstantCommand(() -> telemetry.addData("FrontDetected?", distanceSensor.frontSensorTriggered())),
                            new InstantCommand(() -> telemetry.addData("RearDetected?", distanceSensor.rearSensorTriggered())),
                            new InstantCommand(() -> telemetry.addData("ShootDetected?", distanceSensor.shootSensorTriggered())),
                            new InstantCommand(() -> telemetry.update())
                    )
            );
        }
        else if (mode == TeleOpMode.BLUE){
            targetAngle = 4;
            shooterTime = new ElapsedTime();
            drive = new driveSubsystem(hardwareMap, PostStorage.currentPose);
            endgameKickstand = new Endgame_Kickstand(this);
            intakeSubsystem = new Intake_Subsystem(this);
            distanceSensor = new Distance_Sensor(this);
            scoringGate = new Scoring_Gate(this);
            shooterSubsystem = new Shooter_Subsystem(this, Shooter_Subsystem.Team.BLUE);
            lightIndicator = new Light_Indicator(this);
            register(drive, intakeSubsystem, lightIndicator, distanceSensor, scoringGate, shooterSubsystem);

            defaultDriveCommand = new DefaultDriveCommand(drive,
                    driver::getLeftX,
                    driver::getLeftY,
                    driver::getRightX

            );

            slowModeCommand = new SlowModeCommand(drive,
                    driver::getLeftX,
                    driver::getLeftY,
                    driver::getRightX
            );

            autoAimCommand = new AutoAimTurretCommand(
                    shooterSubsystem,
                    targetAngle,
                    this,
                     startShooter,
                    operator::getRightX,
                    AutoAimTurretCommand.Team.Blue
            );

            CommandScheduler.getInstance().run();
            CommandScheduler.getInstance().setDefaultCommand(drive, defaultDriveCommand);
            CommandScheduler.getInstance().setDefaultCommand(shooterSubsystem, autoAimCommand);

            Button driverIntakeFront = new GamepadButton(driver, GamepadKeys.Button.LEFT_BUMPER);
            Button driveSpeedButton = new GamepadButton(driver, GamepadKeys.Button.RIGHT_BUMPER);
            Button driveTelemetry = new GamepadButton(driver, GamepadKeys.Button.BACK);
            Button drivesShoot = new GamepadButton(driver, GamepadKeys.Button.X);
            Button endgameActivate =  new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
            Button endgameDeActivate = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);

            endgameKickstand.setState(Endgame_Kickstand.EndgameState.UP);

            endgameActivate.whenPressed(new MoveEndgameKickstandCommand(endgameKickstand, Endgame_Kickstand.EndgameState.DOWN));

            endgameDeActivate.whenPressed(new MoveEndgameKickstandCommand(endgameKickstand, Endgame_Kickstand.EndgameState.UP));

            driveSpeedButton
                    .whenHeld(slowModeCommand)
                    .whenReleased(defaultDriveCommand);

            driveTelemetry.whenPressed(
                    new ConditionalCommand(
                            new ParallelCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetryon = false)
                            ),
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clear()),
                                    new InstantCommand(()-> telemetry.addLine("Reminder (Stand Behind The Robot): \n")),
                                    new InstantCommand(()-> telemetry.addLine("Left on Left Joystick = Outwards On Right, Inwards on Left \n")),
                                    new InstantCommand(()-> telemetry.addLine("Right on Left Joystick = Inwards On Right, Outwards on Left \n")),
                                    new InstantCommand(()-> telemetry.addLine("Left on Right Joystick = Forward On Right, Backward on Left \n")),
                                    new InstantCommand(()-> telemetry.addLine("Right on Right Joystick = Backward On Right, Forward on Left \n")),
                                    new InstantCommand(()-> telemetry.update()),
                                    new InstantCommand(()-> telemetryon = true)
                            ),
                            () -> {return (telemetryon);}

                    )
            );

            driverIntakeFront.whileHeld(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetry.addLine("Intake Artifacts From Front")),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(

                                    new InstantCommand(()-> {
                                        if (shooterSensorTriggered == false) {
                                            shooterSensorTriggered = distanceSensor.shootSensorTriggered();
                                        }
                                        if (rearSensorTriggered == false) {
                                            rearSensorTriggered = distanceSensor.rearSensorTriggered() && shooterSensorTriggered;
                                        }
                                        if (frontSensorTriggered == false) {
                                            frontSensorTriggered = distanceSensor.frontSensorTriggered() && rearSensorTriggered && shooterSensorTriggered;
                                        }
                                    }),
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.CLOSE),
                                    new ConditionalCommand(
                                            new ParallelCommandGroup(
                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                                                    new ActuateLightIndicatorCommand(lightIndicator, Light_Indicator.LightIndicatorState.BLUE)
                                            ),
                                            new ConditionalCommand(
                                                    new ParallelCommandGroup(
                                                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.FORWARD_OFF_OFF)
                                                    ),
                                                    new ConditionalCommand(
                                                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.FORWARD_SLOWFORWARD_OFF),
                                                            new ParallelCommandGroup(
                                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.FORWARD_FORWARD_REVERSE),
                                                                    new ActuateLightIndicatorCommand(lightIndicator, Light_Indicator.LightIndicatorState.YELLOW)
                                                            ),
                                                            () -> {return shooterSensorTriggered;}
                                                    ),
                                                    () -> {return (rearSensorTriggered);}
                                            ),
                                            () -> {return (frontSensorTriggered);}
                                    )
                            )
                    )
            ).whenReleased(
                    new SequentialCommandGroup(
                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                            new InstantCommand(()-> {
                                shooterSensorTriggered = false;
                                frontSensorTriggered = false;
                                rearSensorTriggered = false;
                            })
                    )
            );

            drivesShoot.whenPressed(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetry.addLine("Close Range Shooting")),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.OPEN),
                                    new WaitCommand(CLOSE_SHOOT_GATE_OPEN_DELAY),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.OFF_SLOWFORWARD_REVERSE),
                                    new WaitCommand(CLOSE_SHOOT_FIRST_BALL_DELAY),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.THREE_QUARTERS_SHOOT),
                                    new WaitCommand(2000),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT)


                            )
                    )
            );

            Button intakeFront = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
            Button intakeBack = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);
            Button intakeShoot1 = new GamepadButton(operator, GamepadKeys.Button.RIGHT_BUMPER);
            Button shooterFar = new GamepadButton(operator, GamepadKeys.Button.LEFT_BUMPER);
            Button shooterStart = new GamepadButton(operator, GamepadKeys.Button.DPAD_UP);
            Button shooterStartClose = new GamepadButton(operator, GamepadKeys.Button.DPAD_DOWN);
            Button shooterStartCloseClose = new GamepadButton(operator, GamepadKeys.Button.DPAD_LEFT);
            Button shooterStop = new GamepadButton(operator, GamepadKeys.Button.DPAD_RIGHT);
            Button sensorTele = new GamepadButton(operator, GamepadKeys.Button.BACK);
            Button directIntakeShoot = new GamepadButton(operator, GamepadKeys.Button.X);

            scoringGate.setState(Scoring_Gate.ScoringGState.CLOSE);

            intakeFront.whileHeld(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> {
                                        if (shooterSensorTriggered == false) {
                                            shooterSensorTriggered = distanceSensor.shootSensorTriggered();
                                        }
                                        if (rearSensorTriggered == false) {
                                            rearSensorTriggered = distanceSensor.rearSensorTriggered() && shooterSensorTriggered;
                                        }
                                        if (frontSensorTriggered == false) {
                                            frontSensorTriggered = distanceSensor.frontSensorTriggered() && rearSensorTriggered && shooterSensorTriggered;
                                        }
                                    }),
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.CLOSE),
                                    new ConditionalCommand(
                                            new ParallelCommandGroup(
                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                                                    new ActuateLightIndicatorCommand(lightIndicator, Light_Indicator.LightIndicatorState.BLUE)
                                            ),
                                            new ConditionalCommand(
                                                    new ParallelCommandGroup(
                                                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.FORWARD_OFF_OFF)
                                                    ),
                                                    new ConditionalCommand(
                                                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.FORWARD_SLOWFORWARD_OFF),
                                                            new ParallelCommandGroup(
                                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.FORWARD_FORWARD_REVERSE),
                                                                    new ActuateLightIndicatorCommand(lightIndicator, Light_Indicator.LightIndicatorState.YELLOW)
                                                            ),
                                                            () -> {return shooterSensorTriggered;}
                                                    ),
                                                    () -> {return (rearSensorTriggered);}
                                            ),
                                            () -> {return (frontSensorTriggered);}
                                    )
                            )
            ).whenReleased(
                    new SequentialCommandGroup(
                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                            new InstantCommand(()-> {
                                shooterSensorTriggered = false;
                                frontSensorTriggered = false;
                                rearSensorTriggered = false;
                            })
                    )
            );

            intakeBack.whileHeld(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetry.addLine("Intake Artifacts From Back")),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> {
                                        if (frontSensorTriggered == false){
                                            frontSensorTriggered = distanceSensor.frontSensorTriggered();
                                        }
                                        if (shooterSensorTriggered == false) {
                                            shooterSensorTriggered = distanceSensor.shootSensorTriggered();
                                        }
                                        if (rearSensorTriggered == false) {
                                            rearSensorTriggered = distanceSensor.rearSensorTriggered() && shooterSensorTriggered;
                                        }
                                    }),
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.CLOSE),
                                    new ConditionalCommand(
                                            new ParallelCommandGroup(
                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                                                    new ActuateLightIndicatorCommand(lightIndicator, Light_Indicator.LightIndicatorState.BLUE)
                                            ),
                                            new ConditionalCommand(
                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.OFF_OFF_SLOWREVERSE),
                                                    new ConditionalCommand(
                                                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.OFF_OFF_REVERSE),
                                                            new ParallelCommandGroup(
                                                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.OFF_REVERSE_REVERSE),
                                                                    new ActuateLightIndicatorCommand(lightIndicator, Light_Indicator.LightIndicatorState.YELLOW)
                                                            ),
                                                            () -> {return frontSensorTriggered;}
                                                    ),
                                                    ()-> {return shooterSensorTriggered;}
                                            ),
                                            () -> {return (rearSensorTriggered);}
                                    )
                            )
                    )

            ).whenReleased(
                    new SequentialCommandGroup(
                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT),
                            new InstantCommand(()-> {
                                shooterSensorTriggered = false;
                                frontSensorTriggered = false;
                                rearSensorTriggered = false;
                            })

                    )
            );

            intakeShoot1.whenPressed(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetry.addLine("Close Range Shooting")),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.OPEN),
                                    new WaitCommand(CLOSE_SHOOT_GATE_OPEN_DELAY),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.OFF_SLOWFORWARD_REVERSE),
                                    new WaitCommand(CLOSE_SHOOT_FIRST_BALL_DELAY),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.THREE_QUARTERS_SHOOT),
                                    new WaitCommand(2000),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT)
                            )
                    )
            );

            directIntakeShoot.whileHeld(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetry.addLine("Shoot while Intaking")),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.OPEN),
                                    new WaitCommand(250),
                                    new ParallelCommandGroup(
                                            new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.CLOSESHOOT_SHOOT_SHOOT)
                                    )
                            )
                    )
            );

            shooterFar.whenPressed(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new InstantCommand(()-> telemetry.clearAll()),
                                    new InstantCommand(()-> telemetry.addLine("Far Range Shooting")),
                                    new InstantCommand(()-> telemetry.update())
                            ),
                            new SequentialCommandGroup(
                                    new MoveScoringGateCommand(scoringGate, Scoring_Gate.ScoringGState.OPEN),
                                    new WaitCommand(FAR_SHOOT_GATE_OPEN_DELAY),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.HALFSHOOT_SHOOT_SHOOT),
                                    new WaitCommand(FAR_SHOOT_FIRST_BALL_DELAY),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.HALFSHOOT_SHOOT_SHOOT),
                                    new WaitCommand(2000),
                                    new SpinIntakeSubsystemCommand(intakeSubsystem, Intake_Subsystem.IntakeSubsystemState.INIT)
                            )
                    )
            );

            sensorTele.whileHeld(
                    new SequentialCommandGroup(
                            new InstantCommand(() -> telemetry.clearAll()),
                            new InstantCommand(() -> telemetry.addData("FrontDetected?", distanceSensor.frontSensorTriggered())),
                            new InstantCommand(() -> telemetry.addData("RearDetected?", distanceSensor.rearSensorTriggered())),
                            new InstantCommand(() -> telemetry.addData("ShootDetected?", distanceSensor.shootSensorTriggered())),
                            new InstantCommand(() -> telemetry.update())
                    )
            );
        }

    }
    public void changeShooterState(){
        startShooter = !startShooter;
    }
}
