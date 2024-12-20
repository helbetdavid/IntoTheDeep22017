package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.SubSystem.ExtendNou;

@TeleOp(name = "Test Extender Subsystem")
public class testExtSub extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware mapping for the motor
        DcMotor extendo = hardwareMap.get(DcMotor.class, "extendo");

        // Create the extender subsystem instance
        ExtendNou extenderSubsystem = new ExtendNou(extendo);

        // Set up telemetry for monitoring
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Initialized Extender Subsystem.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Update PID coefficients dynamically (if needed)
            extenderSubsystem.updatePID();

            // Run the subsystem to reach the target position
            extenderSubsystem.runToTarget(100);

            // Telemetry feedback for testing
            telemetry.addData("Current Position", extenderSubsystem.getCurrentPosition());
            telemetry.addData("Target", ExtendNou.target);
            telemetry.addData("Motor Power", extendo.getPower());
            telemetry.update();

            // Optionally, add manual control or test conditions here
            if (gamepad1.a) {
                ExtendNou.target = 100; // Increment target
            } else if (gamepad1.b) {
                ExtendNou.target = 0; // Decrement target
            }
        }
    }
}
