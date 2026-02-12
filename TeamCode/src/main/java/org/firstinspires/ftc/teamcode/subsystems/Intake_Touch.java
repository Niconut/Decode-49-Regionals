package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake_Touch {
    public DigitalChannel Intake_Touch;

    public Intake_Touch(HardwareMap hardwareMap) {
        this.Intake_Touch = hardwareMap.get(DigitalChannel.class, "Intake_Touch");
        this.Intake_Touch.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean getState(){
        return Intake_Touch.getState();
    }
}
