package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends SubsystemBase {

    private PIDFController controller;
    public double kP = 0.008, kI = 0.007, kD = 0.0003, kF = 0.000096; // testam sa vedem daca rezolvam overshootul
    public double target = 0;
    private DcMotor motorStanga, motorDreapta,rightFront;
    private final Telemetry telemetry;

    public Lift(DcMotor stanga, DcMotor dreapta,DcMotor rightFront, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.motorDreapta = dreapta;
        this.motorStanga = stanga;
        this.rightFront = rightFront;
        this.controller = new PIDFController(kP, kI, kD, kF);

//        this.motorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.motorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        this.motorStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        this.motorDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updatePIDFGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
        controller.setF(kF);
    }


    public void setTarget(double target) {
        this.target = target;
    }

    public void setPower() {
        double position = rightFront.getCurrentPosition();
        double output = controller.calculate(position, target);

        motorStanga.setPower(-output);
        motorDreapta.setPower(output);

        telemetry.addData("target", target);
        telemetry.addData("pos", position);
        telemetry.update();
    }

    public void setPower(double power) {
        double position = rightFront.getCurrentPosition();

        motorDreapta.setPower(power);
        motorStanga.setPower(-power);

        telemetry.addData("target", target);
        telemetry.addData("pos", position);
        telemetry.update();
    }

    public double getPosition(){
        return rightFront.getCurrentPosition();
    }

    private boolean isMovingToTarget = false; // Track if the motor is moving
    private double targetPosition = 0.0;      // Store the target position

    // Initialize the motion to a target position
    public void startLiftToPosition(double targetPosition) {
        this.targetPosition = targetPosition;
        isMovingToTarget = true; // Indicate that the motion is in progress
    }

    // Update motor control during periodic updates
    public void updateLiftToPosition() {
        if (!isMovingToTarget) {
            return; // Do nothing if not actively moving to target
        }

        double currentPosition = rightFront.getCurrentPosition();

        // Check if the target is reached
        if (Math.abs(targetPosition - currentPosition) <= 10) {
            motorDreapta.setPower(0); // Stop the motor
            motorStanga.setPower(0); // Stop the motor
            isMovingToTarget = false; // Mark as complete
            telemetry.addData("Reached Target", currentPosition);
            return;
        }

        // Otherwise, drive toward the target
        motorDreapta.setPower(controller.calculate(currentPosition, targetPosition));
        motorStanga.setPower(controller.calculate(currentPosition, targetPosition));
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Target Position", targetPosition);
    }

}