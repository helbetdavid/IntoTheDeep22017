package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

@Config
@TeleOp
public class textExtSum extends LinearOpMode {
    public static double target =0;
    private HwMap hwMap;
    private Extend extend;
    private LimeLight limeLight;
    private ServoCam servoCam;
    double tx=0;
    double ty=0;
    @Override
    public void runOpMode() {
        // Initialize hardware and telemetry
        hwMap = new HwMap();
        hwMap.init(hardwareMap);

        extend = new Extend(hwMap.extendo, telemetry);
        limeLight = new LimeLight(hwMap.limelight, telemetry);
        servoCam = new ServoCam(hwMap.servoCam, limeLight);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        waitForStart();

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.a && !previousGamepad1.a) {
                extend.setTarget(200);
                extend.setPower();
            }

            if (currentGamepad1.b && !previousGamepad1.b) {
                servoCam.setAngle(0.5);  // Set servo angle
            }

            // Display telemetry
            limeLight.logPipelineData();
            telemetry.update();
        }
    }

}

