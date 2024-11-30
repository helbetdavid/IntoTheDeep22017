package org.firstinspires.ftc.teamcode.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;
public class ServoCam extends SubsystemBase {
    private final Servo servo;
    private final LimeLight limelight;


    public ServoCam(Servo servo, LimeLight limelight) {
        this.servo = servo;
        this.limelight = limelight;
    }

    public void trackTarget() {
        double angle;
        angle = limelight.getAngle();
        setAngle(angle/180);
    }
    public void setAngle(double angle) {
        servo.setPosition(angle);
    }
}
