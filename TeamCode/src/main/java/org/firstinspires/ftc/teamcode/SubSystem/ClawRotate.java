package org.firstinspires.ftc.teamcode.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawRotate extends SubsystemBase {
    private Servo clawRotate;

    public ClawRotate(Servo clawRotate) {
        this.clawRotate = clawRotate;
    }

    public void rotate(double position) {
        clawRotate.setPosition(position);
    }

    public void rotateUp() {
        clawRotate.setPosition(0.45);
    }
    public void rotateInit(){clawRotate.setPosition(0);
    }

    public void rotateDown() {
        clawRotate.setPosition(1);
    }
}
