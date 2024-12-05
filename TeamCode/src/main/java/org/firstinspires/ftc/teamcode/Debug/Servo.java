package org.firstinspires.ftc.teamcode.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwMap;

@TeleOp
@Config
@Disabled
public class Servo extends LinearOpMode {
    private HwMap hwmap;
    public static double target =0;
    @Override
    public void runOpMode() throws InterruptedException {

        hwmap = new HwMap();
        hwmap.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            hwmap.servoCam.setPosition(target); //aici schimbi servo-ul
            telemetry.addData("Target Position", target);
            telemetry.update();
        }
    }
}
