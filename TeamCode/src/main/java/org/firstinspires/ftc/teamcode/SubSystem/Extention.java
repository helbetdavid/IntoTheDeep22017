package org.firstinspires.ftc.teamcode.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.HwMap;

public class Extention extends SubsystemBase {
    HwMap hwMap = new HwMap();
    // Define hardware
    public Extention(HwMap hwMap) {
        this.hwMap = hwMap;
    }
}
