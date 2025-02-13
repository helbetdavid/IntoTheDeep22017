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
import org.firstinspires.ftc.teamcode.SubSystem.ExtendNou;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

@TeleOp
@Config
public class TeleOpMariusAlbastru extends LinearOpMode {

    ElapsedTime timer;

    public enum RobotState {
        Neutral,
        Scuipa,
        CollectingSubmersibleMetal,
        CollectingSubmersiblePlastic,
        AlignSample,
        RetractCollectingSubmersible,
        RetractCollectingSubmersbilePlastic,
        CollectingGate,
        RetractCollectingGate,
        ScoringSubmersible,
        RetractScoringSubmersible,
        ScoringBasket,
        RetractScoringBasket,
        Manual
    }

    PIDController pidSasiu;
    private HwMap hwMap;
    RobotState robotState = RobotState.Neutral;
    private FtcDashboard dash = FtcDashboard.getInstance();
    public static double ticks = 0;
    public static double ticksext = 0;
    public static double targetExt = 0;
    double targetLift = 0;
    double cond = 0;
    public static double kPsasiu = 0.00045, kIsasiu = 0.000000007, kDsasiu = 0.000000004;

    Claw claw;
    ClawRotate clawRotate;
    Lift lift;
    LimeLight limeLight;
    ServoCam servoCam;

    DcMotor leftFront, leftBack, rightBack, rightFront;

    double xCam, yCam, xReal, yReal;

    @Override
    public void runOpMode() throws InterruptedException {

        timer = new ElapsedTime();

        double targetPosition = 0;
        double manualRotate = 0;
        double manualCam = 0;
        pidSasiu = new PIDController(kPsasiu, kIsasiu, kDsasiu);
        boolean done = false;
        boolean open = false;

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
        lift = new Lift(hwMap.leftLift, hwMap.rightLift, hwMap.rightFront, telemetry);
        limeLight = new LimeLight(hwMap.limelight, telemetry);
        servoCam = new ServoCam(hwMap.servoCam, limeLight);

        limeLight.setPipeline(1);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            lift.setPower();
            extenderSubsystem.runToTarget(targetExt);
            telemetry.addData("State", robotState);
            telemetry.update();

            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

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
                    targetExt = 0;
                    cond = 0;
                    if (gamepad2.dpad_left) {
                        robotState = RobotState.Manual;
                    } else if (gamepad2.a) {
                        targetExt = 200;
                        robotState = RobotState.CollectingSubmersibleMetal;
                    } else if (gamepad2.b) {
                        robotState = RobotState.CollectingGate;
                    } else if (gamepad2.x) {
                        robotState = RobotState.ScoringSubmersible;
                    } else if (gamepad2.y) {
                        robotState = RobotState.ScoringBasket;
                    } else if (gamepad2.dpad_up) {
                        robotState = RobotState.Scuipa;
                        timer.reset();
                    } else if (gamepad2.dpad_right) {
                        robotState = RobotState.CollectingSubmersiblePlastic;
                    }
                    done = false;
                    break;

                case Scuipa:
                    targetExt = 200;
                    clawRotate.rotateDown();
                    if (timer.milliseconds() > 500) {
                        claw.open();
                    }
                    if (timer.milliseconds() > 700) {
                        robotState = RobotState.Neutral;
                    }
                    break;

                case CollectingSubmersibleMetal:
                    claw.openSum();
                    clawRotate.rotateDown();
                    servoCam.straight();
                    lift.setTarget(750);
                    xCam = limeLight.getTargetTx();
                    yCam = limeLight.getTargetTy();
//
                    xReal = - (Math.tan(Math.toRadians(limeLight.getTargetTx())) * ((12 / 384.5) * lift.getPosition()));
                    yReal = - (Math.tan(Math.toRadians(limeLight.getTargetTy())) * ((12 / 384.5) * lift.getPosition()));

                    ticks = xReal * 341.3;
                    ticksext = (yReal - 2.8) * 11.76;

                    if (gamepad2.left_bumper) {
                        targetExt -= 3;
                    }
                    if (gamepad2.right_bumper) {
                        targetExt += 3;
                    }

                    targetPosition = perp.getCurrentPosition() + ticks;

                    if (gamepad2.start) {
                        servoCam.trackTarget();
                        robotState = RobotState.AlignSample;
                    }
                    break;

                case CollectingSubmersiblePlastic:
                    claw.openSum();
                    clawRotate.rotateDown();
                    servoCam.straight();
                    lift.setTarget(500);
                    xCam = limeLight.getTargetTx();
                    yCam = limeLight.getTargetTy();
//
                    xReal = - (Math.tan(Math.toRadians(limeLight.getTargetTx())) * ((12 / 384.5) * lift.getPosition()));
                    yReal = - (Math.tan(Math.toRadians(limeLight.getTargetTy())) * ((12 / 384.5) * lift.getPosition()));

                    ticks = xReal * 341.3;
                    ticksext = (yReal - 2.8) * 11.76;

                    if (gamepad2.left_bumper) {
                        targetExt -= 3;
                    }
                    if (gamepad2.right_bumper) {
                        targetExt += 3;
                    }

                    targetPosition = perp.getCurrentPosition() + ticks;

                    if (gamepad2.start) {
                        servoCam.trackTarget();
                        robotState = RobotState.AlignSample;
                    }
                    break;

                case AlignSample:
                    while ((!gamepad2.dpad_down && Math.abs(perp.getCurrentPosition() - targetPosition) > 150) && !isStopRequested()) {
                        double power = pidSasiu.calculate(perp.getCurrentPosition() - targetPosition);
                        hwMap.rightFront.setPower(-power);
                        hwMap.leftFront.setPower(power);
                        hwMap.rightBack.setPower(power);
                        hwMap.leftBack.setPower(-power);
                    }
                    hwMap.rightFront.setPower(0);
                    hwMap.leftFront.setPower(0);
                    hwMap.rightBack.setPower(0);
                    hwMap.leftBack.setPower(0);
                    sleep(200);
                    if (gamepad2.dpad_down || Math.abs(perp.getCurrentPosition() - targetPosition) < 150) {
                        robotState = RobotState.RetractCollectingSubmersible;
                        targetExt = extenderSubsystem.getCurrentPosition() + ticksext;
                        timer.reset();
                    }
                    if (gamepad2.left_bumper) {
                        targetExt -= 3;
                    }
                    if (gamepad2.right_bumper) {
                        targetExt += 3;
                    }
                    break;

                case RetractCollectingSubmersible:
                    lift.setTarget(0);
                    if (gamepad2.dpad_down && timer.milliseconds() > 500) {
                        claw.close();
                        sleep(50);
                        clawRotate.rotateBasket();
                        targetExt = 0;
                        robotState = RobotState.RetractCollectingSubmersbilePlastic;
                    }
                    if (gamepad2.left_bumper) {
                        targetExt -= 3;
                    }
                    if (gamepad2.right_bumper) {
                        targetExt += 3;
                    }
                    break;

                case RetractCollectingSubmersbilePlastic:
                    clawRotate.rotateBasket();
                    if (gamepad2.dpad_down) {
                        robotState = RobotState.Neutral;
                    }
                    if (gamepad2.left_bumper) {
                        targetExt -= 3;
                    }
                    if (gamepad2.right_bumper) {
                        targetExt += 3;
                    }
                    break;

                case CollectingGate:
                    claw.open();
                    clawRotate.rotateBasket();
                    servoCam.straight();
                    lift.setTarget(220);
                    if (gamepad2.start) {
                        timer.reset();
                        robotState = RobotState.RetractCollectingGate;
                    }
                    if (gamepad2.left_bumper) {
                        targetExt -= 3;
                    }
                    if (gamepad2.right_bumper) {
                        targetExt += 3;
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
                    lift.setTarget(1200);
                    if (gamepad2.start) {
                        targetExt = 350;
                        robotState = RobotState.RetractScoringSubmersible;
                    }
                    break;

                case RetractScoringSubmersible:
                    if (Math.abs(extenderSubsystem.getCurrentPosition() - 350) <= 30 && !done) {
                        claw.open();
                        done = true;
                    }
                    if (done)
                        targetExt = 0;
                    if (gamepad2.dpad_down) {
                        lift.setTarget(0);
                        robotState = RobotState.Neutral;
                    }
                    break;

                case ScoringBasket:
                    lift.setTarget(3150);
                    if (gamepad2.right_stick_button) {
                        clawRotate.rotateBasketFull();
                        cond = 1;
                        robotState = RobotState.RetractScoringBasket;
                        timer.reset();
                    }
                    if (gamepad2.left_trigger>70) {
                        targetLift -= 5;
                    }
                    if (gamepad2.right_trigger>70) {
                        targetLift += 5;
                    }
                    if (gamepad2.start) {
                        robotState = RobotState.RetractScoringBasket;
                        timer.reset();
                    }
                    break;

                case RetractScoringBasket:
                    if (cond==0)
                        clawRotate.rotateBasket();
                    if (timer.milliseconds() > 300)
                        claw.open();
                    if (gamepad2.dpad_down) {
                        clawRotate.rotateInit();
                        sleep(200);
                        robotState = RobotState.Neutral;
                    }
                    break;

                case Manual:
                    lift.setTarget(targetLift);
                    if (gamepad2.left_bumper) {
                        targetLift -= 10;
                    }
                    if (gamepad2.right_bumper) {
                        targetLift += 10;
                    }
                    if (gamepad2.left_trigger > 0.6) {
                        targetExt -= 1;
                    }
                    if (gamepad2.right_trigger > 0.6) {
                        targetExt += 1;
                    }
                    if (gamepad2.a) {
                        open = !open;
                    }
                    if (open) {
                        claw.open();
                    } else claw.close();
                    if (gamepad2.left_stick_y > 0.2) {
                        manualRotate += 0.1;
                        clawRotate.setPosition(manualRotate);
                    } else if (gamepad2.left_stick_y < 0.2) {
                        manualRotate += 0.1;
                        clawRotate.setPosition(manualRotate);
                    }
                    if (gamepad2.left_stick_button) {
                        clawRotate.rotateInit();
                    }
                    if (gamepad2.left_stick_y > 0.2) {
                        manualRotate += 0.1;
                        clawRotate.setPosition(manualRotate);
                    } else if (gamepad2.left_stick_y < 0.2) {
                        manualRotate += 0.1;
                        clawRotate.setPosition(manualRotate);
                    }
                    if (gamepad2.right_stick_button) {
                        servoCam.straight();
                    }
                    if (gamepad2.dpad_down) {
                        robotState = RobotState.Neutral;
                    }
                    break;
            }
        }
    }
}
