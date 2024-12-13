package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystem.ClawRotate;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class FormulaCamera extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private PIDController controller;
    public static double kP = 0.008, kI = 0.00001 , kD = 0.0001;
    public static double target = 0;

    public static double const1 =18.1;
//    public static double const2 = 28.175;
    public static double const2 = 0;
    double inches = 0;
    double ticks =0;
    double anglecam;
    ClawRotate clawRotate;

    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(kP, kI, kD);

        HwMap hwMap = new HwMap();
        hwMap.init(hardwareMap);

        DcMotor extendo = hardwareMap.get(DcMotor.class, "extendo");
        Servo servoCam = hardwareMap.get(Servo.class,"servoCam");
        DcMotor perp = hardwareMap.get(DcMotor.class, "perp");
        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);



        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        Extend extend = new Extend(hwMap.extendo, telemetry);
        Lift lift = new Lift(hwMap.leftLift, hwMap.rightLift, telemetry);

        LimeLight limeLight = new LimeLight(hwMap.limelight, telemetry);


        double xCam, yCam, xReal, yReal;
        double finalTargetExt;
        double target1;

        while(opModeInInit() && !isStopRequested()) {
            lift.setTarget(750);
            lift.setPower();
        }

        clawRotate = new ClawRotate(hwMap.clawRotator);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            TelemetryPacket packet = new TelemetryPacket();


//            limeLight.logPipelineData();
            clawRotate.rotateDown();

//            sleep(3000);
//
            xCam = limeLight.getTargetTx();
            yCam = limeLight.getTargetTy();
//
            xReal = 0.45 * xCam + 0.135;
            yReal = 0.46 * yCam + 0.23;

            inches = 0.3937 * xReal;
            ticks = inches /0.0010494745962278;
//
//
//            servoCam.trackTarget();
//            sleep(5000);
//
            finalTargetExt = 1300-((yReal/11.3)*537.7);
//            extend.setTarget(1300-((yReal/11.3)*537.7));
//            extend.setPower();
//            if(gamepad1.a){
//                servoCam.straight();
//            }
//            else if(gamepad1.b){
//                servoCam.trackTarget();
//
//            }
            controller.setP(kP);
            controller.setI(kI);
            controller.setD(kD);
            double pos = extendo.getCurrentPosition();
            double calcul = controller.calculate(pos,target);
            double anglecam  = limeLight.getAngle();


            target1 = const1*yCam+const2;


            if(gamepad1.a){
                target = extend.getPosition()+target1;
            }
            if(gamepad1.b){
                servoCam.setPosition(anglecam/180);
            }
            else if(gamepad1.start){
                servoCam.setPosition(0.5);
            }

            if (gamepad1.dpad_down) {
                // Calculate the target position
                double targetPosition = perp.getCurrentPosition() + ticks;

                // Loop until the encoder value is within ±300 ticks of the target
                while (Math.abs(perp.getCurrentPosition() - targetPosition) > 1000 && !isStopRequested()) {
                    double error = targetPosition - perp.getCurrentPosition();

                    // Determine motor power based on the error direction
                    double power = 0.4 * Math.signum(error); // Positive for right, negative for left

                    // Set motor powers for strafing
                    hwMap.rightFront.setPower(-power);
                    hwMap.leftFront.setPower(power);
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
            }


            extendo.setPower(calcul);
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);

            telemetry.addData("xCam", xCam);
//            telemetry.addData("yCam", yCam);
            telemetry.addData("xReal", xReal);
            telemetry.addData("target", target1);
            telemetry.addData("inches",inches);
            telemetry.addData("tickes",ticks);
            telemetry.addData("enco", perp.getCurrentPosition() );
            telemetry.addData("taret", extend.getPosition()+target1);
            telemetry.update();
        }
    }
}
