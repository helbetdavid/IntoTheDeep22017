package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;

@TeleOp
public class subsystemext extends LinearOpMode {
    HwMap hwMap;
    Extend extend;


    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HwMap();
        hwMap.init(hardwareMap);
        extend = new Extend(hwMap.extendo,this.telemetry);
        waitForStart();
        extend.setTarget(200);

        while(opModeIsActive()){
            extend.setPower();
        }

    }
}
