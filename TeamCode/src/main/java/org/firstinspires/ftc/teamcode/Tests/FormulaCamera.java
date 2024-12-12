package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

@TeleOp
public class FormulaCamera extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HwMap hwMap = new HwMap();
        hwMap.init(hardwareMap);

        Extend extend = new Extend(hwMap.extendo, telemetry);
        Lift lift = new Lift(hwMap.leftLift, hwMap.rightLift, telemetry);

        LimeLight limeLight = new LimeLight(hwMap.limelight, telemetry);

        ServoCam servoCam = new ServoCam(hwMap.servoCam, limeLight);

        double xCam, yCam, xReal, yReal;

        while(opModeInInit() && !isStopRequested()) {
            extend.setTarget(1300);
            extend.setPower();
            lift.setTarget(750);
            lift.setPower();
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            limeLight.logPipelineData();
//            servoCam.straight();
//            sleep(3000);
//
//            xCam = limeLight.getTargetTx();
//            yCam = limeLight.getTargetTy();
//
//            xReal = 0.45 * xCam + 0.135;
//            yReal = 0.46 * yCam + 0.23;
//
//
//            servoCam.trackTarget();
//            sleep(5000);
//
//            extend.setTarget(1300-((yReal/11.3)*537.7));
//            extend.setPower();

//            telemetry.addData("xCam", xCam);
//            telemetry.addData("yCam", yCam);
//            telemetry.addData("xReal", xReal);
//            telemetry.addData("yReal", yReal);
            telemetry.update();
        }
    }
}
