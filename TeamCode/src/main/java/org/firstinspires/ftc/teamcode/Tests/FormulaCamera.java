package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;

public class FormulaCamera extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HwMap hwMap = new HwMap();
        hwMap.init(hardwareMap);

        LimeLight limeLight = new LimeLight(hwMap.limelight, telemetry);

        double xCam, yCam, xReal, yReal;;
        waitForStart();
        while (opModeIsActive()) {
            xCam = limeLight.getTargetTx();
            yCam = limeLight.getTargetTy();

            xReal = 0.45 * xCam + 0.135;
            yReal = 0.46 * yCam + 0.23;

            telemetry.addData("xCam", xCam);
            telemetry.addData("yCam", yCam);
            telemetry.addData("xReal", xReal);
            telemetry.addData("yReal", yReal);
            telemetry.update();
        }
    }
}
