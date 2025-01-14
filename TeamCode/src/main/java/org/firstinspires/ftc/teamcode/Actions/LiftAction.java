package org.firstinspires.ftc.teamcode.Actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;

public class LiftAction {
    private final Lift lift;

    public LiftAction(HardwareMap hardwareMap, Telemetry telemetry) {
        HwMap hwMap = new HwMap();
        hwMap.init(hardwareMap);
        this.lift = new Lift(hwMap.leftLift, hwMap.rightLift, telemetry);
    }
    public Action liftToPosition(double targetPosition) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                double currentPosition = lift.getPosition();

                // Set the target position
                lift.setTarget(targetPosition);
                // Stop when the target is reached
                if (Math.abs(targetPosition - currentPosition) <= 20 ) {
                    lift.setTarget(currentPosition);
                    lift.setPower(); // Maintain position
                    telemetryPacket.put("Reached Target", currentPosition);
                    return false; // Action completed
                }

                // Otherwise, drive toward the target
                lift.setPower();
                telemetryPacket.put("Current Position", currentPosition);
                telemetryPacket.put("Target Position", targetPosition);
                return true; // Action still running
            }
        };
    }

}