  package org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.PostStorage;
  public class Turret_Analog_Input_Action extends SubsystemBase {
    private AnalogInput turretAnalog;

    private double currentEncoderAngle = 0;
    private double lastEncoderAngle = 0;
    private int encoderRotations = 0;
    private double totalEncoderAngle = 0;
    private double delta = 0;
    private double totalTurretAngle = 0;
    private final double MAX_VOLTAGE = 3.3;
    private final double TWO_PI = 2.0 * Math.PI;
    private final double HALF_PI = 0.5 * Math.PI;

    // Use 5.812 if that is your exact physical gear ratio.
    private final double GEAR_RATIO = 5.812;

    public Turret_Analog_Input_Action(HardwareMap hardwareMap){
        turretAnalog = hardwareMap.get(AnalogInput.class, "turretFeedback");
    }

    public class getInitialAngle implements Action{
        private boolean initialized = false;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized){
                currentEncoderAngle = (turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI; // goes from 0 to 2 PI
                delta = currentEncoderAngle - lastEncoderAngle;
                lastEncoderAngle = currentEncoderAngle;
                // wrap angles
                if (delta < -Math.PI) {         // Wrapped forward
                    encoderRotations++;
                }
                else if (delta > Math.PI){      // Wrapped backward
                    encoderRotations--;
                }
                // Total angler (Radians) the SERVO has spun
                totalEncoderAngle = (encoderRotations * TWO_PI) + currentEncoderAngle;
                totalTurretAngle = (totalEncoderAngle / GEAR_RATIO);

                PostStorage.initalAutoTurretPos = totalTurretAngle;
                initialized = true;
            }
            return false;
        }
    }

    public class sendTotalTurretAngle implements Action{
        private boolean initialized = false;
        double turretBearing;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized){
                // angle limit
                currentEncoderAngle = (turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI; // goes from 0 to 2 PI
                delta = currentEncoderAngle - lastEncoderAngle;
                lastEncoderAngle = currentEncoderAngle;
                // wrap angles
                if (delta < -Math.PI) {         // Wrapped forward
                    encoderRotations++;
                }
                else if (delta > Math.PI){      // Wrapped backward
                    encoderRotations--;
                }
                // Total angler (Radians) the SERVO has spun
                totalEncoderAngle = (encoderRotations * TWO_PI) + currentEncoderAngle;
                totalTurretAngle = (totalEncoderAngle / GEAR_RATIO);

                PostStorage.endAutoTurretPos = totalTurretAngle;
            }
            return true;
        }
    }


    public Action GetInitialAngle(){return new getInitialAngle();}
    public Action GetTotalTurretAngle(){return new sendTotalTurretAngle();}





}
