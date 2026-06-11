  package org.firstinspires.ftc.teamcode.subsystems.scoring.scoring_actions;

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

  public class Shooter_Subsystem_Action extends SubsystemBase {
    public PIDController PID;
    private Limelight3A limelight;

    private CRServo servoTurret;
    public static double kp = 0.02 ;
    public static double ki = 0.0 ;
    public static double kd = 0.0 ;

    private double targettx;
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
    private double startEncoderAngle = 0;

    // Use 5.812 if that is your exact physical gear ratio.
    private final double GEAR_RATIO = 5.812;

    public enum Pipeline {
        RED,
        BLUE
    }

      private final Pipeline team;

    public Shooter_Subsystem_Action(HardwareMap hardwareMap, Pipeline team){
        this.team = team;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        servoTurret = hardwareMap.get(CRServo.class, "ScoringTurret");    //  port 0
        servoTurret.setDirection(CRServo.Direction.FORWARD);
        turretAnalog = hardwareMap.get(AnalogInput.class, "turretFeedback");
        PID = new PIDController(kp, ki, kd);
        if (team == Pipeline.RED){
            limelight.pipelineSwitch(1);
        } else if (team == Pipeline.BLUE){
            limelight.pipelineSwitch(0);
        }
    }

    public void setPower(double power){
        servoTurret.setPower(power);
    }



    public class autoAim implements Action{
        private boolean initialized = false;
        double turretBearing;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized){
                LLResult result = limelight.getLatestResult();

                double bearingAprilTag = result.getTx();
                double rangeAprilTag = result.getTy();

                double[] turretProp =  {bearingAprilTag, rangeAprilTag};
                turretBearing = (turretProp[0]);
                double turretRange = (turretProp[1]);
                if (team == Pipeline.BLUE) {
                    targettx = 0;
                } else if (team == Pipeline.RED){
                    targettx = -0.5;
                }

                double error = turretBearing - targettx;

                double power = PID.calculate(-turretBearing, targettx);

                // angle limit
                currentEncoderAngle = ((turretAnalog.getVoltage() - startEncoderAngle) / MAX_VOLTAGE) * TWO_PI; // goes from 0 to 2 PI
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

                //kp * error;
                if ((totalTurretAngle > 0.5 && power > 0) || (totalTurretAngle < -0.5 && power < 0)){
                    servoTurret.setPower(0);
                }else{
                    servoTurret.setPower(power);
                }
            }
            return true;
        }
    }

    public class INIT implements Action{
        public boolean initialized = false;
        public double INIT = 0;
        @Override
        public boolean run (@NonNull TelemetryPacket packet){
            if(!initialized){
                servoTurret.setPower(INIT);
                initialized = true;
            }
            return false;
        }
    }

      public class readTurretPosition implements Action{
          public boolean initialized = false;
          public double INIT = 0;
          @Override
          public boolean run (@NonNull TelemetryPacket packet){
              //if(!initialized){
              //startEncoderAngle = (turretAnalog.getVoltage() / MAX_VOLTAGE) * TWO_PI;
              startEncoderAngle = turretAnalog.getVoltage();

              //initialized = true;
             // }
              return false;
          }
      }

    public Action AutoAim(){return new autoAim();}
    public Action TurretINIT(){ return new INIT();}
    public Action ReadTurretPosition(){ return new readTurretPosition();}



}
