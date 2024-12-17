package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;

public class ExtendAction {
    private final Extend extend;

    public ExtendAction(HardwareMap hardwareMap, Telemetry telemetry) {
        HwMap hwMap = new HwMap();
        hwMap.init(hardwareMap);
        this.extend = new Extend(hwMap.extendo, telemetry);
    }

    public Action extendToPosition(double targetPosition) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double currentPosition = extend.getPosition();

                // Set the target position
                extend.setTarget(targetPosition);

                // Stop when the target is reached
                if (Math.abs(targetPosition - currentPosition) <= 45) {
                    extend.setPower(0); // Stop the motor
                    telemetryPacket.put("Reached Target", currentPosition);
                    return false; // Action completed
                }

                // Otherwise, drive toward the target
                extend.setPower();
                telemetryPacket.put("Current Position", currentPosition);
                telemetryPacket.put("Target Position", targetPosition);
                return true; // Action still running
            }
        };
    }
}
