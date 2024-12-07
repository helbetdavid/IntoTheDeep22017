package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class testlift extends LinearOpMode {
    public static double target =0.4;
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor dreapta = hardwareMap.get(DcMotor.class, "rightLift");
        DcMotor stanga = hardwareMap.get(DcMotor.class,"perp");

        stanga.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        while (opModeIsActive()) {
            dreapta.setPower(target);
            stanga.setPower(target);
            telemetry.update();
        }

    }
}
