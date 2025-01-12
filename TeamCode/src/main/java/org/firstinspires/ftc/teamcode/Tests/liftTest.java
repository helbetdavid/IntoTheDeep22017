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
    private PIDController controllercam;
    private LimeLight limeLight;
    private HwMap hwMap;
    public static double kP = 0.006, kI=0, kD=0, kF=0.0001;
    public static double target=0;

    public static double kPcam = 0, kIcam=0, kDcam=0;
    public static double targetcam=30;





    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDFController(kP,kI,kD,kF);
        controllercam = new PIDController(kPcam,kIcam,kDcam);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        DcMotor rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        DcMotor leftLift = hardwareMap.get(DcMotor.class, "perp");

        ClawAction clawAction = new ClawAction(hardwareMap);
        ClawRotateAction clawRotateAction = new ClawRotateAction(hardwareMap);
        ServoCamAction servoCamAction = new ServoCamAction(hardwareMap);
        ExtendAction extendAction = new ExtendAction(hardwareMap, this.telemetry);
        LiftAction liftAction = new LiftAction(hardwareMap, this.telemetry);

        DcMotor extendo = hardwareMap.get(DcMotor.class, "extendo");
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        leftLift.setDirection(DcMotorSimple.Direction.REVERSE);

        hwMap = new HwMap();
        hwMap.init(hardwareMap);
        limeLight = new LimeLight(hwMap.limelight, telemetry);
        limeLight.setPipeline(0);

        waitForStart();

        while(opModeIsActive()){
            controller.setPIDF(kP,kI,kD,kF);
            controllercam.setPID(kPcam,kIcam,kDcam);

            double poscam = limeLight.getTargetArea();
            double calculcam = controllercam.calculate(poscam,targetcam);

            double pos = rightLift.getCurrentPosition();
            double calcul = controller.calculate(pos,target);
//            Actions.runBlocking(
//                    new SequentialAction(
//                            clawAction.clawOpen(),
//                            clawRotateAction.clawRotateDown(),
//                            servoCamAction.straight()
//                    )
//            );

            rightLift.setPower(calcul);
            leftLift.setPower(calcul);

            extendo.setPower(calculcam);




            telemetry.addData("Target Area", targetcam);
            telemetry.addData("Current Area", limeLight.getTargetArea());
            telemetry.update();


        }

    }
}
