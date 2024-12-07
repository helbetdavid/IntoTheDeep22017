package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.SubSystem.Extend;

public class ExtendAction {

    private Extend extender;
    public double position;

    public ExtendAction(Extend extender) {
        this.extender = extender;
    }

    public Action getPosition() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                position = extender.getPosition();
                return false;
            }
        };
    }

    public Action updatePIDGains(double kP, double kI, double kD) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                extender.updatePIDGains(kP, kI, kD);
                // telemetry bla bla
                return false;
            }
        };
    }

    public Action setPower() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                extender.setPower();
                return false;
            }
        };
    }

    public Action setTarget(double target) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                extender.target = target;
                telemetryPacket.put("Set target to: ", target);
                return false;
            }
        };
    }
}
