package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;

@Config
@TeleOp
public class TestLiftSub extends LinearOpMode {
    HwMap hwMap;
    Lift lift;
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HwMap();
        hwMap.init(hardwareMap);
        lift = new Lift(hwMap.leftLift, hwMap.rightLift, this.telemetry);

        lift.setTarget(target);
        waitForStart();

        while(opModeIsActive()){
            lift.setPower();
        }
    }
}
