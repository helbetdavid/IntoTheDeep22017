package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Extend extends SubsystemBase {
    private final PIDController controller;
    private DcMotor extendo;
    private final Telemetry telemetry;
    public double kP = 0.045, kI = 0, kD = 0.0007;
    public double target = 0;

    private boolean isMovingToTarget = false; // Track if the motor is moving
    private double targetPosition = 0.0;      // Store the target position


    public Extend(DcMotor extendo, Telemetry telemetry) {
        this.extendo = extendo;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.controller = new PIDController(kP, kI, kD);

        this.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void updatePIDGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setPower() {
        double currentPosition = extendo.getCurrentPosition();
        double output = controller.calculate(currentPosition, target);
        extendo.setPower(output);

        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Output", output);
        telemetry.update();
    }

    public void setPower(double power) {
        double currentPosition = extendo.getCurrentPosition();
        extendo.setPower(power);

        telemetry.addData("Current Position", currentPosition);
        telemetry.update();
    }

    public double getPosition(){
        return extendo.getCurrentPosition();
    }

    // Initialize the motion to a target position
    public void startExtendToPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        isMovingToTarget = true; // Indicate that the motion is in progress
    }

    // Update motor control during periodic updates
    public void updateExtendToPosition() {
        if (!isMovingToTarget) {
            return; // Do nothing if not actively moving to target
        }

        double currentPosition = extendo.getCurrentPosition();

        // Check if the target is reached
        if (Math.abs(targetPosition - currentPosition) <= 10) {
            extendo.setPower(0); // Stop the motor
            isMovingToTarget = false; // Mark as complete
            telemetry.addData("Reached Target", currentPosition);
            return;
        }

        // Otherwise, drive toward the target
        extendo.setPower(controller.calculate(currentPosition, targetPosition));
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Target Position", targetPosition);
    }

}
