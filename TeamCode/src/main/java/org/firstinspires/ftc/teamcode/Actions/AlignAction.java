package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;

public class AlignAction implements Action {
    private final HwMap hwMap;
    private final Telemetry telemetry;
    private final LimeLight limeLight;
    private final double tolerance;
    private final double kStrafe;
    private final double kArmExtension;

    public AlignAction(HwMap hwMap, Telemetry telemetry, LimeLight limeLight, double tolerance, double kStrafe, double kArmExtension) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.limeLight = limeLight;
        this.tolerance = tolerance;
        this.kStrafe = kStrafe;
        this.kArmExtension = kArmExtension;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        // Get the target offsets directly from LimeLight
        double tx = limeLight.getTargetTx();
        double ty = limeLight.getTargetTy();

        // Calculate adjustments based on tx and ty
        double x = Math.abs(tx) > tolerance ? -kStrafe * tx : 0;
        double y = Math.abs(ty) > tolerance ? -kArmExtension * ty : 0;

        // Calculate motor powers
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
        double frontLeftPower = (y + x) / denominator;
        double backLeftPower = (y - x) / denominator;
        double frontRightPower = (y - x) / denominator;
        double backRightPower = (y + x) / denominator;

        // Set motor powers
        hwMap.leftFront.setPower(frontLeftPower);
        hwMap.leftBack.setPower(backLeftPower);
        hwMap.rightFront.setPower(frontRightPower);
        hwMap.rightBack.setPower(backRightPower);

        // Telemetry for debugging
        telemetry.addData("Aligning", "tx: %.2f, ty: %.2f, x: %.2f, y: %.2f", tx, ty, x, y);
        telemetry.update();

        // Alignment is done if both tx and ty are within tolerance
        return Math.abs(tx) <= tolerance && Math.abs(ty) <= tolerance;
    }
}
