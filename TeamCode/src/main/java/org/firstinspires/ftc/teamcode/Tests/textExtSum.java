package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.RR.TankDrive;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;

@Config
@TeleOp
public class textExtSum extends LinearOpMode {
    public static double target =0;
    private HwMap hwMap;
    private Extend extend;
    @Override
    public void runOpMode() {
        // Initialize hardware and telemetry
        hwMap = new HwMap();
        hwMap.init(hardwareMap);

        // Create the Extend subsystem
        extend = new Extend(hwMap.extendo, telemetry);

        // Set a target position

        waitForStart();

        while (opModeIsActive()) {
            // Continuously update motor power
            extend.setTarget(target);
            extend.setPower();

            telemetry.addData("Target Position", target);
            telemetry.addData("Current Position", hwMap.extendo.getCurrentPosition());
            telemetry.update();
        }
    }

}
