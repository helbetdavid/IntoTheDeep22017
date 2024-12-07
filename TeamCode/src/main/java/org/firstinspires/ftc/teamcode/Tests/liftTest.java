package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class liftTest extends LinearOpMode {

    private PIDFController controller;
    public static double kP = 0, kI=0, kD=0, kF=0;
    public static double target=0;

    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDFController(kP,kI,kD,kF);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        DcMotor leftLift = hardwareMap.get(DcMotor.class, "perp");

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()){
            controller.setPIDF(kP,kI,kD,kF);
            double pos = rightLift.getCurrentPosition();
            double calcul = controller.calculate(pos,target);

            rightLift.setPower(calcul);
            leftLift.setPower(calcul);

            telemetry.addData("target",target);
            telemetry.addData("pos",pos);
            telemetry.update();
        }
    }
}
