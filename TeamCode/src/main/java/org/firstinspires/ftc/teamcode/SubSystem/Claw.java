package org.firstinspires.ftc.teamcode.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {
    private Servo claw;

    public Claw(Servo claw) {
        this.claw = claw;
    }
    public void open() {
        claw.setPosition(0.5);
    }
    public void close() {
        claw.setPosition(0);
    }

}