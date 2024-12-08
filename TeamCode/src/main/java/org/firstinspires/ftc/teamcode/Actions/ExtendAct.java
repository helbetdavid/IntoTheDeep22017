package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;

public class ExtendAct {

    private HwMap hwMap;
    private Extend extender;
    public double position;
    private Telemetry telemetry;

    public ExtendAct(HardwareMap hardwareMap,Telemetry telemetry) {
        hwMap = new HwMap();
        hwMap.init(hardwareMap);

        extender = new Extend(hwMap.extendo,telemetry);
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
                if (extender.getPosition() < extender.target) {
                    extender.setPower();
                    return true;
                }
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