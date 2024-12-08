package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.SubSystem.ClawRotate;

public class ClawRotateAction {
    private ClawRotate servo;

    public ClawRotateAction(ClawRotate clawRotate) {
        this.servo = clawRotate;
    }

    public Action rotate(double position) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                servo.rotate(position);
                return false;
            }
        };
    }

    public Action goUp() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                servo.rotateUp();
                return false;
            }
        };
    }

    public Action goDown() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                servo.rotateDown();
                return false;
            }
        };
    }
}
