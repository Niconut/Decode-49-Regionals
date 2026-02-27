package org.firstinspires.ftc.teamcode.subsystems.intake.Intake_Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake_Subsystem_Action{
    public DcMotorEx frontIntake;
    public DcMotorEx backIntake;
    public DcMotorEx midIntake;
    public boolean True = true;
    public static boolean shooterSensorTriggered = false;
    public static boolean frontSensorTriggered = false;
    public static boolean rearSensorTriggered = false;
    public ElapsedTime runtime;


    /*public double INIT[] = {0, 0, 0};
        public double[] FRONTINTAKE = {1, 1, 0};
        public double[] BACKINTAKE = {0, -1, -1};
        public double[] SHOOT = {1, 1, -1};*/
    public Intake_Subsystem_Action(HardwareMap hardwareMap){
        frontIntake = hardwareMap.get(DcMotorEx.class, "frontIntake");
        midIntake = hardwareMap.get(DcMotorEx.class, "midIntake");
        backIntake = hardwareMap.get(DcMotorEx.class, "backIntake");
        runtime = new ElapsedTime();
        this.frontIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.midIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backIntake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public enum IntakeMode {
        INIT(0, 0, 0),
        FARSHOOT(0.5, 0.5, -0.6),

        OFF_OFF_FORWARD(0, 0, 1),
        OFF_OFF_REVERSE(0, 0, -1),
        OFF_FORWARD_OFF(0, 1, 0),
        OFF_FORWARD_FORWARD(0, 1, 1),
        OFF_FORWARD_REVERSE(0, 1, -1),
        OFF_REVERSE_OFF(0, -1, 0),
        OFF_REVERSE_FORWARD(0, -1, 1),
        OFF_REVERSE_REVERSE(0, -1, -1),

        FORWARD_OFF_OFF(1, 0, 0),
        FORWARD_OFF_FORWARD(1, 0, 1),
        FORWARD_OFF_REVERSE(1, 0, -1),
        FASTFORWARD_FASTFORWARD_OFF(1,1,0),
        FORWARD_FORWARD_OFF(0.9, 0.9, 0),
        SLOWFORWARD_SLOWFORWARD_OFF(0.7, 0.7, 0),
        FORWARD_FORWARD_FORWARD(1, 1, 1),
        FORWARD_FORWARD_REVERSE(1, 1, -1),
        FORWARD_REVERSE_OFF(1, -1, 0),
        FORWARD_REVERSE_FORWARD(1, -1, 1),
        FORWARD_REVERSE_REVERSE(1, -1, -1),

        REVERSE_OFF_OFF(-1, 0, 0),
        REVERSE_OFF_FORWARD(-1, 0, 1),
        REVERSE_OFF_REVERSE(-1, 0, -1),
        REVERSE_FORWARD_OFF(-1, 1, 0),
        REVERSE_FORWARD_FORWARD(-1, 1, 1),
        REVERSE_FORWARD_REVERSE(-1, 1, -1),
        REVERSE_REVERSE_OFF(-1, -1, 0),
        REVERSE_REVERSE_FORWARD(-1, -1, 1),
        REVERSE_REVERSE_REVERSE(-1, -1, -1);

        public final double front;
        public final double mid;
        public final double back;

        IntakeMode(double front, double mid, double back) {
            this.front = front;
            this.mid   = mid;
            this.back  = back;
        }

        // Create an Action for RoadRunner

    }
    public Action action(IntakeMode mode) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    //while (!initialized) {
                    frontIntake.setPower(mode.front);
                    midIntake.setPower(mode.mid);
                    backIntake.setPower(mode.back);
                }
                /**/
                initialized = true;
                return false; // finish immediately
            }
        };
    }

    /*public Action sensorAction(IntakeMode mode) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    runtime.reset();
                    packet.put("RunTime", runtime);
                    while (!initialized) {
                        frontIntake.setPower(mode.front);
                        midIntake.setPower(mode.mid);
                        backIntake.setPower(mode.back);
                        if(!frontSensorTriggered){
                            frontSensorTriggered = distanceSensor.frontSensorTriggered() && distanceSensor.rearSensorTriggered() && distanceSensor.shootSensorTriggered();
                        }
                        if (!shooterSensorTriggered) {
                            shooterSensorTriggered = distanceSensor.shootSensorTriggered();
                        }
                        if (!rearSensorTriggered) {
                            rearSensorTriggered = distanceSensor.rearSensorTriggered() && shooterSensorTriggered;
                        }
                        if (shooterSensorTriggered){
                            backIntake.setPower(0);
                        }else if(rearSensorTriggered){
                            backIntake.setPower(0);
                            midIntake.setPower(0);
                        }else if(frontSensorTriggered){
                            initialized = true;
                        } else if(runtime.seconds() == 2.50){
                            initialized = true;
                        }
                    }
                }

                //initialized = true;
                return false; // finish immediately
            }
        };
    }

    public Action distanceAction(IntakeMode mode) {
        return new Action() {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    frontIntake.setPower(mode.front);
                    midIntake.setPower(mode.mid);
                    backIntake.setPower(mode.back);
                    initialized = true;
                }
                if(!frontSensorTriggered){
                    frontSensorTriggered = distanceSensor.frontSensorTriggered() && distanceSensor.rearSensorTriggered() && distanceSensor.shootSensorTriggered();
                }
                if (!shooterSensorTriggered) {
                    shooterSensorTriggered = distanceSensor.shootSensorTriggered();
                }
                if (!rearSensorTriggered) {
                    rearSensorTriggered = distanceSensor.rearSensorTriggered() && shooterSensorTriggered;
                }
                if (shooterSensorTriggered){
                    backIntake.setPower(0);
                }else if(rearSensorTriggered){
                    backIntake.setPower(0);
                    midIntake.setPower(0);
                }
                if(runtime.seconds() == 2.50){
                    frontSensorTriggered = true;
                }

                return !frontSensorTriggered ;
            }
        };
    }*/

    private void setPowers(IntakeMode intakeMode){
        frontIntake.setPower(intakeMode.front);
        midIntake.setPower(intakeMode.mid);
        backIntake.setPower(intakeMode.back);
    }


}
