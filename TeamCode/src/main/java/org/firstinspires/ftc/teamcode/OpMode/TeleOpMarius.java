package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Claw;
import org.firstinspires.ftc.teamcode.SubSystem.ClawRotate;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

public class TeleOpMarius extends LinearOpMode {

    public enum RobotState {
        Neutral,
        CollectingSum,
        RetractCollectingSum,
        CollectingGate,
        RetractCollectingGate,
        ScoringSum,
        RetractScoringSum,
        ScoringBasket,
        RetractScoringBasket
    }
    private HwMap hwMap;
    RobotState robotState = RobotState.Neutral;
    private FtcDashboard dash = FtcDashboard.getInstance();
    Claw claw;
    ClawRotate clawRotate;
    Extend extend;
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

        claw = new Claw(hwMap.claw);
        clawRotate = new ClawRotate(hwMap.clawRotator);
        extend = new Extend(hwMap.extendo, telemetry);
        lift = new Lift(hwMap.leftLift, hwMap.rightLift, telemetry);
        limeLight = new LimeLight(hwMap.limelight, telemetry);
        servoCam = new ServoCam(hwMap.servoCam, limeLight);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("State", robotState);
            telemetry.update();

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

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            if(gamepad2.a){
                robotState = RobotState.CollectingSum;
            }
            else if (gamepad2.b) {
                robotState = RobotState.CollectingGate;
            }
            else if (gamepad2.x) {
                robotState = RobotState.ScoringSum;
            }
            else if (gamepad2.y) {
                robotState = RobotState.ScoringBasket;
            }

            switch (robotState){
                case Neutral:
                    claw.open();
                    clawRotate.rotateInit();
                    servoCam.straight();
                    lift.startLiftToPosition(0);
                    extend.startExtendToPosition(0);
                    lift.updateLiftToPosition();
                    extend.updateExtendToPosition();
                    break;

                case CollectingSum:
                    double tx = limeLight.getTargetTx();
                    double ty = limeLight.getTargetTy();

                    claw.open();
                    lift.startLiftToPosition(200);
                    lift.updateLiftToPosition();
                    extend.startExtendToPosition(1200);
                    extend.updateExtendToPosition();
                    clawRotate.rotateDown();
                    servoCam.trackTarget();

                case RetractCollectingSum:

                case CollectingGate:

                case RetractCollectingGate:

                case ScoringSum:

                case RetractScoringSum:

                case ScoringBasket:

                case RetractScoringBasket:

            }
        }

    }
}
