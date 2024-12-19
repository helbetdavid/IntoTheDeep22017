package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Claw;
import org.firstinspires.ftc.teamcode.SubSystem.ClawRotate;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

@TeleOp
@Config
public class TeleOpMariusRosu extends LinearOpMode {

    ElapsedTime timer;

    public enum RobotState {
        Neutral,
        CollectingSubmersible,
        RetractCollectingSubmersible,
        CollectingGate,
        RetractCollectingGate,
        ScoringSubmersible,
        RetractScoringSubmersible,
        ScoringBasket,
        RetractScoringBasket
    }

    PIDController pidSasiu;
    private HwMap hwMap;
    RobotState robotState = RobotState.Neutral;
    private FtcDashboard dash = FtcDashboard.getInstance();
    public static double servo = 0;
    public static double const1 = 3.8;
    public static double const2 = 0;
    public static double inches = 0;
    public static double ticks = 0;
    public static double const3 = 0.45;
    double anglecam;
    public static double kPsasiu = 0.04, kIsasiu = 0.00001, kDsasiu = 0.0001;

    Claw claw;
    ClawRotate clawRotate;
    Extend extend;
    Lift lift;
    LimeLight limeLight;
    ServoCam servoCam;

    DcMotor leftFront, leftBack, rightBack, rightFront;

    double xCam, yCam, xReal, yReal;
    double finalTargetExt;
    double target1;

    @Override
    public void runOpMode() throws InterruptedException {

        timer = new ElapsedTime();

        pidSasiu = new PIDController(kPsasiu, kIsasiu, kDsasiu);
        boolean did = false;
        boolean did1 = false;
        boolean did2 = false;
        boolean done = false; // trebuie ceva mai elegant
        boolean done1 = false;
        boolean done2 = false;
        boolean done3 = false;
        boolean done4 = false;
        boolean done5 = false;
        boolean done6 = false;
        boolean done7 = false;
        boolean mihneaserv = false;

        hwMap = new HwMap();
        hwMap.init(hardwareMap);

        leftFront = hwMap.leftFront;
        leftBack = hwMap.leftBack;
        rightBack = hwMap.rightBack;
        rightFront = hwMap.rightFront;

        DcMotor perp = hardwareMap.get(DcMotor.class, "perp");

        claw = new Claw(hwMap.claw);
        clawRotate = new ClawRotate(hwMap.clawRotator);
        extend = new Extend(hwMap.extendo, telemetry);
        lift = new Lift(hwMap.leftLift, hwMap.rightLift, telemetry);
        limeLight = new LimeLight(hwMap.limelight, telemetry);
        servoCam = new ServoCam(hwMap.servoCam, limeLight);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            extend.setPower();
            lift.setPower();

            telemetry.addData("State", robotState);
            telemetry.update();

            xCam = limeLight.getTargetTx();
            yCam = limeLight.getTargetTy();
//
            xReal = const3 * xCam; //0.135

            inches = 0.3937 * xReal;
            ticks = inches / 0.0010494745962278;

            target1 = const1 * yCam + const2;

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

            if (gamepad2.right_trigger > 0.1) {
                claw.setPosition(0.4);
            }

            if (gamepad1.right_bumper) {
                frontLeftPower *= 0.3;
                backLeftPower *= 0.3;
                frontRightPower *= 0.3;
                backRightPower *= 0.3;
            }

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);


            switch (robotState) {
                case Neutral:
                    clawRotate.rotateInit();
                    servoCam.straight();
                    lift.setTarget(0);
                    extend.setTarget(0);
                    if (gamepad2.a) {
                        robotState = RobotState.CollectingSubmersible;
                    } else if (gamepad2.b) {
                        robotState = RobotState.CollectingGate;
                    } else if (gamepad2.x) {
                        robotState = RobotState.ScoringSubmersible;
                    } else if (gamepad2.y) {
                        robotState = RobotState.ScoringBasket;
                    }
                    done = false;
                    done1 = false;
                    done2 = false;
                    did2 = false;
                    done3 = false;
                    done4 = false;
                    done5 = false;
                    done6 = false;
                    done7 = false;

                    break;

                case CollectingSubmersible:
                    claw.openSum();
                    clawRotate.rotateDown();
                    servoCam.straight();
                    extend.setTarget(200);
                    lift.setTarget(750);

                    if (gamepad2.start) {
                        robotState = RobotState.RetractCollectingSubmersible;
                    }
                    break;

                case RetractCollectingSubmersible:
                    // Intai ne miscam pe x
                    double targetPosition = perp.getCurrentPosition() + ticks;

                    // Loop until the encoder value is within ±300 ticks of the target
                    while (Math.abs(perp.getCurrentPosition() - targetPosition) > 1100 && !isStopRequested() && !done3) {
                        double error = targetPosition - perp.getCurrentPosition();

                        // Determine motor power based on the error direction
                        double power = 0.4 * Math.signum(error); // Positive for right, negative for left

                        // Set motor powers for strafing
                        hwMap.rightFront.setPower(-power * 1.1);
                        hwMap.leftFront.setPower(power * 1.1);
                        hwMap.rightBack.setPower(power);
                        hwMap.leftBack.setPower(-power);

                        // Telemetry for debugging
                        telemetry.addData("Current Position", perp.getCurrentPosition());
                        telemetry.addData("Target Position", targetPosition);
                        telemetry.addData("Error", error);
                        telemetry.update();
                    }
                    hwMap.rightFront.setPower(0);
                    hwMap.leftFront.setPower(0);
                    hwMap.rightBack.setPower(0);
                    hwMap.leftBack.setPower(0);
                    done3 = true;
                    telemetry.addData("ycam", yCam);
                    telemetry.addData("xcam", xCam);
                    telemetry.update();
                    if (!done4) {
                        extend.setTarget(extend.getPosition() + target1);
                        done4 = true;
                    }
                    if (!done5) {
                        servoCam.trackTarget();
                        done5 = true;
                    }
                    if (done5 && !done6) {
                        sleep(500);
                        lift.setTarget(0);
                        done6 = true;
                    }
                    if (Math.abs(lift.getPosition()) <= 20) {
                        sleep(200);
                        claw.close();
                        done7 = true;
                    }
                    if (done7) {
                        clawRotate.rotateInit();
                        done7 = false;
                    }
                    if (gamepad2.dpad_down) {
                        robotState = RobotState.Neutral;
                    }
                    break;

                case CollectingGate:
                    claw.open();
                    clawRotate.rotateUp();
                    servoCam.straight();
                    lift.setTarget(150);
                    if (gamepad2.start) {
                        timer.reset();
                        robotState = RobotState.RetractCollectingGate;
                    }
                    break;

                case RetractCollectingGate:
                    claw.close();
                    if (timer.milliseconds() > 300)
                        lift.setTarget(700);
                    if (gamepad2.dpad_down) {
                        robotState = RobotState.Neutral;
                    }
                    break;

                case ScoringSubmersible:
                    claw.close();
                    clawRotate.rotateUp();
                    servoCam.straight();
                    lift.setTarget(2300);
                    if (gamepad2.start) {
                        robotState = RobotState.RetractScoringSubmersible;
                    }
                    break;

                case RetractScoringSubmersible:
                    // nu ma mai obosesc sa schimb, oricum trb modificat pt prins sampleuri orizontal
                    // va rog eu sa gasiti o modalitate sa nu mai faceti cu done
                    lift.setTarget(0);
                    if (Math.abs(lift.getPosition() - 1700) <= 30 && !done) {
                        claw.open();
                        done = true;
                    }
                    if (gamepad2.dpad_down) {
                        robotState = RobotState.Neutral;
                    }
                    break;

                case ScoringBasket:
                    lift.setTarget(4450);
                    if (gamepad2.start) {
                        robotState = RobotState.RetractScoringBasket;
                        timer.reset();
                    }
                    break;

                case RetractScoringBasket:
                    clawRotate.rotateUp();
                    if (timer.milliseconds() > 200)
                        claw.open();
                    if (timer.milliseconds() > 400)
                        clawRotate.rotateInit();

//                    if (Math.abs(lift.getPosition() - 4450) <= 50 && !done2) {
//                        sleep(200);
//                        claw.open();
//                        sleep(200);
//                        done2 = true;
//                        clawRotate.rotateInit();
//                    }

                    if (gamepad2.dpad_down) {
                        robotState = RobotState.Neutral;
                    }
                    break;
            }
        }

    }
}
