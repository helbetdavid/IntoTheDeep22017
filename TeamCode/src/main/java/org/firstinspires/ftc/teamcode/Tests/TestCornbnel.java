package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Claw;
import org.firstinspires.ftc.teamcode.SubSystem.ClawRotate;
import org.firstinspires.ftc.teamcode.SubSystem.ExtendNou;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

@Config
@TeleOp
public class TestCornbnel extends LinearOpMode {
    private HwMap hwMap;
    Claw claw;
    ClawRotate clawRotate;
    Lift lift;
    LimeLight limeLight;
    ServoCam servoCam;

    DcMotor leftFront, leftBack, rightBack, rightFront;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HwMap();
        hwMap.init(hardwareMap);

        leftFront = hwMap.leftFront;
        leftBack = hwMap.leftBack;
        rightBack = hwMap.rightBack;
        rightFront = hwMap.rightFront;

        DcMotor perp = hardwareMap.get(DcMotor.class, "perp");

        claw = new Claw(hwMap.claw);
        clawRotate = new ClawRotate(hwMap.clawRotator);
        ExtendNou extenderSubsystem = new ExtendNou(hwMap.extendo);
        lift = new Lift(hwMap.leftLift, hwMap.rightLift, telemetry);
        limeLight = new LimeLight(hwMap.limelight, telemetry);
        servoCam = new ServoCam(hwMap.servoCam, limeLight);

        limeLight.setPipeline(1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            lift.setPower();

            if (gamepad2.x) {
                claw.openSum();
                clawRotate.rotateDown();
                servoCam.straight();
                lift.setTarget(750);
            }

            telemetry.addData("ticks perp: ", perp.getCurrentPosition());
            telemetry.addData("ticks extend: ", extenderSubsystem.getCurrentPosition());
            telemetry.addData("x(cm): ", Math.tan(Math.toRadians(limeLight.getTargetTx())) * 23);
            telemetry.addData("y(cm): ", Math.tan(Math.toRadians(limeLight.getTargetTy())) * 23);
            telemetry.update();
        }
    }
}
