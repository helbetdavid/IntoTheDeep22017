package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;

@TeleOp
public class Luatdepegardusor extends LinearOpMode {
    private HwMap hwMap;
    private LimeLight limeLight;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
 //519539 Eucodez2025
        hwMap = new HwMap();
        hwMap.init(hardwareMap);
        limeLight = new LimeLight(hwMap.limelight, telemetry);
        limeLight.setPipeline(0);
        waitForStart();
        while (opModeIsActive()) {



            telemetry.addData("targetarea", limeLight.getTargetArea());
            telemetry.update();
        }
    }
}
