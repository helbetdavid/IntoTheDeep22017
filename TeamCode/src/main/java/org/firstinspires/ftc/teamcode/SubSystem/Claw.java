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
        claw.setPosition(0.65);
    }
    public void close() {
        claw.setPosition(0.4);
    }
    //inainte era 0.44

    public void openSum(){
        claw.setPosition(0.54);
    }
    public void openAuto(){
        claw.setPosition(0.71);
    }



}
