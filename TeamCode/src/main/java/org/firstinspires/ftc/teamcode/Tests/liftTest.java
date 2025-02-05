package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Actions.ClawAction;
import org.firstinspires.ftc.teamcode.Actions.ClawRotateAction;
import org.firstinspires.ftc.teamcode.Actions.ExtendAction;
import org.firstinspires.ftc.teamcode.Actions.LiftAction;
import org.firstinspires.ftc.teamcode.Actions.ServoCamAction;
import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;

@TeleOp
@Config
public class liftTest extends LinearOpMode {

    private PIDFController controller;
    private HwMap hwMap;
    public static double kP = 0.01, kI=0, kD=0, kF=0;
    public static double target=0;

    public static double stanga=0;
    public static double dreapta=0;





    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDFController(kP,kI,kD,kF);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor rightLift = hardwareMap.get(DcMotor.class, "par2");
        DcMotor leftLift = hardwareMap.get(DcMotor.class, "perp");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        ClawAction clawAction = new ClawAction(hardwareMap);
        ClawRotateAction clawRotateAction = new ClawRotateAction(hardwareMap);
        ServoCamAction servoCamAction = new ServoCamAction(hardwareMap);
        ExtendAction extendAction = new ExtendAction(hardwareMap, this.telemetry);
        LiftAction liftAction = new LiftAction(hardwareMap, this.telemetry);

        DcMotor extendo = hardwareMap.get(DcMotor.class, "extendo");
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        hwMap = new HwMap();
        hwMap.init(hardwareMap);

        waitForStart();
        double done =0;
        while(opModeIsActive()){
            controller.setPIDF(kP, kI, kD, kF);
            double pos = rightFront.getCurrentPosition();
            double calcul = controller.calculate(pos, target);

            rightLift.setPower(calcul);
            leftLift.setPower(-calcul);




            telemetry.addData("Target Area", target);
            telemetry.addData("Current Area", rightFront.getCurrentPosition());
            telemetry.update();


        }

    }
}