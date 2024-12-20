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
    private PIDFController controller;
    public static double kP = 0.045, kI = 0 , kD = 0.0007,kF =0;
    public static int target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDFController(kP, kI, kD,kF);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotor extendo = hardwareMap.get(DcMotor.class, "extendo");
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()){
            controller.setP(kP);
            controller.setI(kI);
            controller.setD(kD);
            controller.setF(kF);
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
