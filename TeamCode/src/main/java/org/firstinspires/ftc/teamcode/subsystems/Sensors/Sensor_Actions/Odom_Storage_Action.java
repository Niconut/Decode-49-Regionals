  package org.firstinspires.ftc.teamcode.subsystems.Sensors.Sensor_Actions;

  import androidx.annotation.NonNull;

  import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
  import com.acmerobotics.roadrunner.Action;
  import com.arcrobotics.ftclib.command.SubsystemBase;
  import com.qualcomm.robotcore.hardware.AnalogInput;
  import com.qualcomm.robotcore.hardware.HardwareMap;

  import org.firstinspires.ftc.teamcode.subsystems.PostStorage;
  import org.firstinspires.ftc.teamcode.teamcode.MecanumDrive;

  public class Odom_Storage_Action extends SubsystemBase {
    private AnalogInput turretAnalog;
    private MecanumDrive drive;

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

    public Odom_Storage_Action(HardwareMap hardwareMap, MecanumDrive drive){
        turretAnalog = hardwareMap.get(AnalogInput.class, "turretFeedback");
        this.drive = drive;
    }

    public class sendOdomCoords implements Action{
        private boolean initialized = false;
        double turretBearing;
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if(!initialized){
                PostStorage.currentPose = drive.localizer.getPose();
            }
            return true;
        }
    }


    public Action sendOdomCoords(){return new sendOdomCoords();}





}
