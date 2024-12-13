package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.ClawRotate;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

@TeleOp
@Config
public class FormulaCamera extends LinearOpMode {
    private PIDController controller;
    public static double kP = 0.008, kI = 0.00001 , kD = 0.0001;
    public static double target = 0;
    double anglecam;


    ClawRotate clawRotate;

    @Override
    public void runOpMode() throws InterruptedException {
        controller = new PIDController(kP, kI, kD);

        HwMap hwMap = new HwMap();
        hwMap.init(hardwareMap);

        DcMotor extendo = hardwareMap.get(DcMotor.class, "extendo");
        Servo servoCam = hardwareMap.get(Servo.class,"servoCam");


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
//            limeLight.logPipelineData();
            clawRotate.rotateDown();

//            sleep(3000);
//
//            xCam = limeLight.getTargetTx();
            yCam = limeLight.getTargetTy();
//
//            xReal = 0.45 * xCam + 0.135;
            yReal = 0.46 * yCam + 0.23;
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


            target1 = 18.33*yCam+28.175;


            if(gamepad1.a){
                target = extend.getPosition()+target1;
            }
            if(gamepad1.b){
                servoCam.setPosition(anglecam/180);
            }
            else if(gamepad1.start){
                servoCam.setPosition(0.5);
            }


            extendo.setPower(calcul);

//            telemetry.addData("xCam", xCam);
//            telemetry.addData("yCam", yCam);
//            telemetry.addData("xReal", xReal);
            telemetry.addData("target", target1);
            telemetry.addData("target", extend.getPosition()+target1);
            telemetry.update();
        }
    }
}
