package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.*;

@TeleOp
@Config
public class TeleOpChatGPT extends LinearOpMode {

    // Enums
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

    // Constants
    public static double servo = 0;
    public static double inches = 0;
    public static double ticks = 0;
    public static double ticksext = 0;
    public static double const3 = 0.48;
    public static double targetExt = 0;
    public static double kPsasiu = 0.04, kIsasiu = 0.00001, kDsasiu = 0.0001;

    // Subsystems
    private HwMap hwMap;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private Claw claw;
    private ClawRotate clawRotate;
    private Lift lift;
    private LimeLight limeLight;
    private ServoCam servoCam;
    private PIDController pidSasiu;

    // Motors
    private DcMotor leftFront, leftBack, rightBack, rightFront, perp;

    // Variables
    private ElapsedTime timer;
    private RobotState robotState = RobotState.Neutral;
    private double xCam, yCam, xReal, yReal;
    private boolean[] stateFlags = new boolean[10];

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            lift.setPower();
            updateExtender();
            updateTelemetry();
            driveRobot();

            handleState(robotState);
        }
    }

    private void initHardware() {
        timer = new ElapsedTime();
        pidSasiu = new PIDController(kPsasiu, kIsasiu, kDsasiu);
        hwMap = new HwMap();
        hwMap.init(hardwareMap);

        leftFront = hwMap.leftFront;
        leftBack = hwMap.leftBack;
        rightBack = hwMap.rightBack;
        rightFront = hwMap.rightFront;
        perp = hardwareMap.get(DcMotor.class, "perp");

        claw = new Claw(hwMap.claw);
        clawRotate = new ClawRotate(hwMap.clawRotator);
        lift = new Lift(hwMap.leftLift, hwMap.rightLift, telemetry);
        limeLight = new LimeLight(hwMap.limelight, telemetry);
        servoCam = new ServoCam(hwMap.servoCam, limeLight);

        limeLight.setPipeline(1);
    }

    private void updateExtender() {
        double targetPosition = perp.getCurrentPosition() + ticks;
        if (Math.abs(targetPosition - perp.getCurrentPosition()) > 1100 && !stateFlags[3]) {
            adjustMotorPower(targetPosition);
            stateFlags[3] = true;
        }
    }

    private void updateTelemetry() {
        telemetry.addData("State", robotState);
        telemetry.update();
    }

    private void driveRobot() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.1;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

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
    }

    private void handleState(RobotState state) {
        switch (state) {
            case Neutral:
                resetToNeutral();
                break;
            case CollectingSubmersible:
                executeCollectingSubmersible();
                break;
            case RetractCollectingSubmersible:
                executeRetractCollectingSubmersible();
                break;
            case CollectingGate:
                executeCollectingGate();
                break;
            case RetractCollectingGate:
                executeRetractCollectingGate();
                break;
            case ScoringSubmersible:
                executeScoringSubmersible();
                break;
            case RetractScoringSubmersible:
                executeRetractScoringSubmersible();
                break;
            case ScoringBasket:
                executeScoringBasket();
                break;
            case RetractScoringBasket:
                executeRetractScoringBasket();
                break;
        }
    }

    private void resetToNeutral() {
        clawRotate.rotateInit();
        servoCam.straight();
        lift.setTarget(0);
        targetExt = 0;

        if (gamepad2.a) robotState = RobotState.CollectingSubmersible;
        if (gamepad2.b) robotState = RobotState.CollectingGate;
        if (gamepad2.x) robotState = RobotState.ScoringSubmersible;
        if (gamepad2.y) robotState = RobotState.ScoringBasket;

        resetStateFlags();
    }

    private void resetStateFlags() {
        for (int i = 0; i < stateFlags.length; i++) stateFlags[i] = false;
    }

    private void adjustMotorPower(double targetPosition) {
        double error = targetPosition - perp.getCurrentPosition();
        double power = 0.4 * Math.signum(error);

        hwMap.rightFront.setPower(-power * 1.05);
        hwMap.leftFront.setPower(power * 1.05);
        hwMap.rightBack.setPower(power);
        hwMap.leftBack.setPower(-power);
    }

    private void executeCollectingSubmersible() {
        claw.openSum();
        clawRotate.rotateDown();
        servoCam.straight();
        targetExt = 200;
        lift.setTarget(750);
        if (gamepad2.start) robotState = RobotState.RetractCollectingSubmersible;
    }

    private void executeRetractCollectingSubmersible() {
        claw.close();
        if (Math.abs(lift.getPosition() - 350) <= 30 && !stateFlags[0]) {
            claw.open();
            stateFlags[0] = true;
        }
        if (stateFlags[0]) targetExt = 0;
        if (gamepad2.dpad_down) {
            lift.setTarget(0);
            robotState = RobotState.Neutral;
        }
    }

    private void executeCollectingGate() {
        claw.open();
        clawRotate.rotateBasket();
        servoCam.straight();
        lift.setTarget(250);
        if (gamepad2.start) robotState = RobotState.RetractCollectingGate;
    }

    private void executeRetractCollectingGate() {
        claw.close();
        if (timer.milliseconds() > 300) lift.setTarget(700);
        if (gamepad2.dpad_down) robotState = RobotState.Neutral;
    }

    private void executeScoringSubmersible() {
        claw.close();
        clawRotate.rotateUp();
        servoCam.straight();
        lift.setTarget(1460);
        if (gamepad2.start) {
            targetExt = 350;
            robotState = RobotState.RetractScoringSubmersible;
        }
    }

    private void executeRetractScoringSubmersible() {
        if (Math.abs(lift.getPosition() - 350) <= 30 && !stateFlags[0]) {
            claw.open();
            stateFlags[0] = true;
        }
        if (stateFlags[0]) targetExt = 0;
        if (gamepad2.dpad_down) {
            lift.setTarget(0);
            robotState = RobotState.Neutral;
        }
    }

    private void executeScoringBasket() {
        lift.setTarget(4450);
        if (gamepad2.start) {
            robotState = RobotState.RetractScoringBasket;
            timer.reset();
        }
    }

    private void executeRetractScoringBasket() {
        clawRotate.rotateBasket();
        if (timer.milliseconds() > 200) claw.open();
        if (timer.milliseconds() > 400) clawRotate.rotateInit();
        if (gamepad2.dpad_down) robotState = RobotState.Neutral;
    }
}
