package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HwMap;

public class AlignAct implements Action {
    private final HwMap hwMap;
    private final Telemetry telemetry;
    private final double tolerance;
    private final double kStrafe;
    private final double kArmExtension;
    private final double tx, ty;

    public AlignAct(HwMap hwMap, Telemetry telemetry, double tx, double ty, double tolerance, double kStrafe, double kArmExtension) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
        this.tolerance = tolerance;
        this.kStrafe = kStrafe;
        this.kArmExtension = kArmExtension;
        this.tx = tx;
        this.ty = ty;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        // Calculate strafing adjustment based on tx (horizontal alignment)
        double x = Math.abs(tx) > tolerance ? -kStrafe * tx : 0;

        // Calculate arm extension adjustment based on ty (vertical alignment)
        double armExtension = Math.abs(ty) > tolerance ? kArmExtension * ty : 0;

        // Calculate motor powers for strafing
        double denominator = Math.max(Math.abs(x), 1);
        double frontLeftPower = x / denominator;
        double backLeftPower = x / denominator;
        double frontRightPower = -x / denominator;
        double backRightPower = -x / denominator;

        // Set strafing motor powers
        hwMap.leftFront.setPower(frontLeftPower);
        hwMap.leftBack.setPower(backLeftPower);
        hwMap.rightFront.setPower(frontRightPower);
        hwMap.rightBack.setPower(backRightPower);

        // Set arm extension power (assuming the arm is controlled by the extendo motor)
        hwMap.extendo.setPower(armExtension);

        // Update Driver Station telemetry for debugging
        telemetry.addData("Aligning", Math.abs(tx) <= tolerance && Math.abs(ty) <= tolerance ? "Aligned" : "Aligning");
        telemetry.addData("tx", tx);
        telemetry.addData("ty", ty);
        telemetry.addData("x adjustment", x);
        telemetry.addData("arm extension", armExtension);
        telemetry.addData("Tolerance", tolerance);
        telemetry.update();

        // Update FTC Dashboard telemetry packet
        telemetryPacket.put("Aligning Status", Math.abs(tx) <= tolerance && Math.abs(ty) <= tolerance ? "Aligned" : "Aligning");
        telemetryPacket.put("tx", tx);
        telemetryPacket.put("ty", ty);
        telemetryPacket.put("x adjustment", x);
        telemetryPacket.put("arm extension", armExtension);
        telemetryPacket.put("Tolerance", tolerance);

        // Return whether the robot is aligned (both tx and ty within tolerance)
        return Math.abs(tx) <= tolerance && Math.abs(ty) <= tolerance;
    }
}
