package org.firstinspires.ftc.teamcode.SubSystem;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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
        double angle=0;
        angle = limelight.getAngle();
        setAngle(angle/180);
    }
    public void setAngle(double angle) {
        servo.setPosition(angle);
    }

    public void straight() {
        servo.setPosition(0.5);
    }

    public void lateral() {
        servo.setPosition(0.0);
    }
}
