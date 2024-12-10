package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Actions.ClawAct;
import org.firstinspires.ftc.teamcode.Actions.ClawRotateAct;
import org.firstinspires.ftc.teamcode.Actions.ExtendAct;
import org.firstinspires.ftc.teamcode.Actions.LiftAct;
import org.firstinspires.ftc.teamcode.Actions.ServoCamAct;

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

        ClawAct clawAct = new ClawAct(hardwareMap);
        ClawRotateAct clawRotateAct = new ClawRotateAct(hardwareMap);
        ServoCamAct servoCamAct = new ServoCamAct(hardwareMap);
        ExtendAct extendAct = new ExtendAct(hardwareMap, this.telemetry);
        LiftAct liftAct = new LiftAct(hardwareMap, this.telemetry);

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while(opModeIsActive()){
            controller.setPIDF(kP,kI,kD,kF);
            double pos = rightLift.getCurrentPosition();
            double calcul = controller.calculate(pos,target);
            Actions.runBlocking(
                    new SequentialAction(
                            clawAct.clawClose(),
                            clawRotateAct.clawRotateUp(),
                            servoCamAct.straight()
                    )
            );

            rightLift.setPower(calcul);
            leftLift.setPower(calcul);

            telemetry.addData("target",target);
            telemetry.addData("pos",pos);
            telemetry.update();
        }
    }
}
