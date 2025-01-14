package org.firstinspires.ftc.teamcode.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawRotate extends SubsystemBase {
    private Servo clawRotate;

    public ClawRotate(Servo clawRotate) {
        this.clawRotate = clawRotate;
    }

    public void setPosition(double position) {
        clawRotate.setPosition(position);
    }

    public void rotateUp() {
        clawRotate.setPosition(0.05);
    }
    public void rotateSpec() {
        clawRotate.setPosition(0.75);
    }

    public void rotateBasket(){
        clawRotate.setPosition(0.44);
    }
    public void rotatepid(){clawRotate.setPosition(0.5);
    }
    public void rotateInit(){
        clawRotate.setPosition(0.1);
    }

    public void rotateSub(){
        clawRotate.setPosition(0.4);
    }

    public void rotateDown() {
        clawRotate.setPosition(0.95);
    }
}
