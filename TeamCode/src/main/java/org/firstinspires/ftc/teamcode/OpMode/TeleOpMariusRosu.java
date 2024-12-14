package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
        CollectingSum,
        RetractCollectingSum,
        CollectingGate,
        RetractCollectingGate,
        ScoringSum,
        RetractScoringSum,
        ScoringBasket,
        RetractScoringBasket,
        Level
    }

    private HwMap hwMap;
    RobotState robotState = RobotState.Neutral;
    private FtcDashboard dash = FtcDashboard.getInstance();
    public static double servo =0;
    public static double const1 =18.33;
    public static double const2 = 0;
    double inches = 0;
    double ticks =0;
    double anglecam;

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
        boolean did = false;
        boolean did1 = false;
        boolean did2 = false;
        boolean done = false; // trebuie ceva mai elegant
        boolean done1= false;
        boolean done2= false;
        boolean done3 = false;
        boolean done4 = false;
        boolean done5=false;
        boolean done6=false;
        boolean done7=false;

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
            xReal = 0.42 * xCam; //0.135

            inches = 0.3937 * xReal;
            ticks = inches /0.0010494745962278;

            target1 = const1*yCam+const2;

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




            if(gamepad1.right_bumper){
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

                    claw.close();

                    clawRotate.rotateInit();

                    servoCam.straight();

                    lift.setTarget(0);

                    if (gamepad2.a) {
                        robotState = RobotState.CollectingSum;
                    } else if (gamepad2.b) {
                        robotState = RobotState.CollectingGate;
                    } else if (gamepad2.x) {
                        robotState = RobotState.ScoringSum;
                    } else if (gamepad2.y) {
                        robotState = RobotState.ScoringBasket;
                    }
                    else if(gamepad2.share){
                        robotState = RobotState.Level;
                    }
                    done = false;
                    done1 =false;
                    done2 = false;
                    did2 = false;
                    done3 =false;
                    done4 = false;
                    done5=false;
                    done6=false;
                    done7=false;

                    break;

                case CollectingSum:
                    claw.openSum();
                    clawRotate.rotateDown();
                    servoCam.straight();
                    if (!did2) {
                        lift.setTarget(750);
                        lift.setPower();
                        extend.setTarget(1000);
//                        extend.setPower();
                        if (Math.abs(extend.getPosition() - 1000) < 10)
                            did2 = true;
                    }
//                    double tx = limeLight.getTargetTx();
//                    double ty = limeLight.getTargetTy();
//
//                    claw.open();
//
//                    lift.setTarget(300);
//                    lift.setPower();
//                    clawRotate.rotateDown();
//                    if(!did){
//                        extend.setTarget(1300);
//                        extend.setPower();
//                        sleep(200);
//                        did=true;
//                    }
//
//                    servoCam.trackTarget();
//
//                    double yr = 0.45*ty+0.135;
//                    extend.setTarget(1300-((yr/11.3)*537.7));
//                    extend.setPower();


//                    lift.startLiftToPosition(200);
//                    lift.updateLiftToPosition();
//                    extend.startExtendToPosition(1200);
//                    extend.updateExtendToPosition();
//                    clawRotate.rotateDown();
////                    servoCam.trackTarget();
//                    lift.setTarget(750);
//                    lift.setPower();
                    if(gamepad2.start){
                        robotState = robotState.RetractCollectingSum;
                    }
                    break;

                case RetractCollectingSum:
//                    claw.close();
//                    lift.startLiftToPosition(0);
//                    lift.updateLiftToPosition();
//                    extend.startExtendToPosition(0);
//                    extend.updateExtendToPosition();
//                    clawRotate.rotateInit();
//                    servoCam.straight();
                    double targetPosition = perp.getCurrentPosition() + ticks;

                    // Loop until the encoder value is within ±300 ticks of the target
                    while (Math.abs(perp.getCurrentPosition() - targetPosition) > 1000 && !isStopRequested() && !done3) {
                        double error = targetPosition - perp.getCurrentPosition();

                        // Determine motor power based on the error direction
                        double power = 0.4 * Math.signum(error); // Positive for right, negative for left

                        // Set motor powers for strafing
                        hwMap.rightFront.setPower(-power*1.1);
                        hwMap.leftFront.setPower(power*1.1);
                        hwMap.rightBack.setPower(power);
                        hwMap.leftBack.setPower(-power);

                        // Telemetry for debugging
                        telemetry.addData("Current Position", perp.getCurrentPosition());
                        telemetry.addData("Target Position", targetPosition);
                        telemetry.addData("Error", error);
                        telemetry.update();

                    }


                    // Stop the motors after reaching the target
                    hwMap.rightFront.setPower(0);
                    hwMap.leftFront.setPower(0);
                    hwMap.rightBack.setPower(0);
                    hwMap.leftBack.setPower(0);
                    done3 =true;
                    telemetry.addData("ycam",yCam);
                    telemetry.addData("xcam",xCam);
                    telemetry.update();
                    if(!done4){
                        extend.setTarget(extend.getPosition()+target1);

                        done4=true;

                    }
                    if(!done5){
                        servoCam.trackTarget();
                        done5=true;
                    }
                    if(done5 && !done6){
                        lift.setTarget(0);
                        done6=true;
                    }
                    if(Math.abs(lift.getPosition())<=20 ){
                        sleep(200);
                        claw.close();
                        done7=true;
                    }
                    if(done7){
                        clawRotate.rotateInit();
                        done7=false;
                    }
                    if(gamepad2.dpad_down){
                        extend.setTarget(0);
                        extend.setPower();
                        robotState = RobotState.Neutral;
                    }
                    break;

                case CollectingGate:
                    claw.open();
                    clawRotate.rotateUp();
                    servoCam.straight();
                    lift.setTarget(150);
//                    lift.setPower();
                    if(gamepad2.start){
                        robotState = RobotState.RetractCollectingGate;
                    }
                    break;

                case RetractCollectingGate:
                    if(!done1){
                        sleep(200);
                        claw.close();
                        done1=true;
                    }
                    lift.setTarget(700);
//                    lift.setPower();
                    if(gamepad2.dpad_down){
                        robotState = RobotState.Neutral;
                    }
                    break;

                case ScoringSum:
                    claw.close();
                    clawRotate.rotateUp();
                    servoCam.straight();
                    lift.setTarget(2300);
//                    lift.setPower();
                    if (gamepad2.start) {
                        robotState = RobotState.RetractScoringSum;
                    }
                    break;

                case RetractScoringSum:
                    timer.reset();
                    lift.setTarget(0);
//                    lift.setPower();
                    if (Math.abs(lift.getPosition() - 1700) <= 30 && !done) {
                        claw.open();
                        done = true;
                    }
                    if(gamepad2.dpad_down){
                        robotState = RobotState.Neutral;
                    }
                    break;


                case ScoringBasket:
                    lift.setTarget(4450);
//                    lift.setPower();
                    if(gamepad2.start){
                        robotState = RobotState.RetractScoringBasket;
                    }
                    break;

                case RetractScoringBasket:
                    clawRotate.rotateUp();

                    if(Math.abs(lift.getPosition() - 4450) <= 50 && !done2){
                        sleep(200);
                        claw.open();
                        sleep(200);
                        done2 = true;
                        clawRotate.rotateInit();
                    }

                    if(gamepad2.dpad_down) {
                        robotState = RobotState.Neutral;
                    }
                    break;
                case Level:
                    lift.setTarget(1500);
                    clawRotate.rotateCollect();
                    if(gamepad2.dpad_down){
                        robotState = RobotState.Neutral;
                    }
                    break;
            }
        }

    }
}
