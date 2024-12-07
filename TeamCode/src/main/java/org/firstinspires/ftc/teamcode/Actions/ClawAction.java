package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.SubSystem.Claw;

public class ClawAction {
    private Claw claw;

    public ClawAction(Claw claw) {
        this.claw = claw;
    }

    public Action open() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.open();
                return false;
            }
        };
    }

    public Action close() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.close();
                return false;
            }
        };
    }

    public Action setPosition(double position) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                claw.setPosition(position);
                return false;
            }
        };
    }
}
