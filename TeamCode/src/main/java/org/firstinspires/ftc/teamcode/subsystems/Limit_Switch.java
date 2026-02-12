package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limit_Switch {
    private Limit_Switch Limit_Switch;

    public Limit_Switch(HardwareMap hardwareMap) {
        this.Limit_Switch = hardwareMap.get(Limit_Switch.class, "Limit_Switch");
    }
}
