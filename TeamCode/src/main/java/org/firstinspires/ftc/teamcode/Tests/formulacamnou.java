package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystem.Claw;
import org.firstinspires.ftc.teamcode.SubSystem.ClawRotate;
import org.firstinspires.ftc.teamcode.SubSystem.ExtendNou;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

@TeleOp
@Config
public class formulacamnou extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    public static double targetExt = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        HwMap hwMap = new HwMap();
        hwMap.init(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        ExtendNou extenderSubsystem = new ExtendNou(hwMap.extendo);
        DcMotor perp = hardwareMap.get(DcMotor.class, "perp");
        Lift lift = new Lift(hwMap.leftLift, hwMap.rightLift, telemetry);
        LimeLight limeLight = new LimeLight(hwMap.limelight, telemetry);
        ClawRotate clawRotate = new ClawRotate(hwMap.clawRotator);
        Claw claw = new Claw(hwMap.claw);
        ServoCam servoCam = new ServoCam(hwMap.servoCam, limeLight);



        limeLight.setPipeline(0);

        double targetnouY=0;


        waitForStart();


        while (opModeIsActive()) {
            if(gamepad2.a){
                targetExt=200;
                lift.setTarget(750);
                claw.open();
                clawRotate.rotateDown();
            }
            lift.setPower();
            extenderSubsystem.updatePID();
            extenderSubsystem.runToTarget(targetExt);

            targetnouY = Math.tan(Math.toRadians(limeLight.getTargetTy()))*23;
            double targetnouX = Math.tan(Math.toRadians(limeLight.getTargetTx()))*23;

            double targetvechiY = 3.9*limeLight.getTargetTy();


            if(gamepad2.b){
                servoCam.setAngle(limeLight.getAngle()/180);
            }
            if (gamepad2.y){
                servoCam.straight();
            }

            telemetry.addData("yCm",targetnouY);
            telemetry.addData("xCM",targetnouX);
            telemetry.update();

        }
    }
}
