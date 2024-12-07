package org.firstinspires.ftc.teamcode.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw extends SubsystemBase {
    private Servo claw;

    public Claw(Servo claw) {
        this.claw = claw;
    }
    public void setPosition(double position){
        claw.setPosition(position);
    }
    public void open() {
        claw.setPosition(0.6);
    }
    public void close() {
        claw.setPosition(0.445);
    }


}
