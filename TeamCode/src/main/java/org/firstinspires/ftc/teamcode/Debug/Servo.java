package org.firstinspires.ftc.teamcode.Debug;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwMap;

@TeleOp
@Config
public class Servo extends LinearOpMode {
    private HwMap hwmap;
    public static double rotat =0;
    public static double cheata =0;
    public static double cam=0;
    @Override
    public void runOpMode() throws InterruptedException {

        hwmap = new HwMap();
        hwmap.init(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            hwmap.clawRotator.setPosition(rotat);
            hwmap.claw.setPosition(cheata);
            hwmap.servoCam.setPosition(cam);
        }
    }
}
