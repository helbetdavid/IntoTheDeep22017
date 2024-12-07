package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class testExt extends LinearOpMode {
    private PIDController controller;
    public static double kP = 0.008, kI = 0.00001 , kD = 0.0001;
    public static int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(kP, kI, kD);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotor extendo = hardwareMap.get(DcMotor.class, "extendo");
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()){
            controller.setP(kP);
            controller.setI(kI);
            controller.setD(kD);
            double pos = extendo.getCurrentPosition();
            double calcul = controller.calculate(pos,target);

            extendo.setPower(calcul);

            telemetry.addData("pos", pos);
            telemetry.addData("target", target);
            telemetry.addData("pos1", extendo.getCurrentPosition());
            telemetry.update();
        }
    }
}
