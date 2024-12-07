package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;

@TeleOp
@Config
public class teleopLift extends LinearOpMode {

    private HwMap hwMap;
    private Lift lift;
    private Extend extend;
    public static double target=0;
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HwMap();
        hwMap.init(hardwareMap);

        lift = new Lift(hwMap.rightLift,hwMap.leftLift,telemetry);
        extend = new Extend(hwMap.extendo,telemetry);

        extend.setTarget(200);

        waitForStart();
        while(opModeIsActive()){

            extend.setPower();
        }
    }
}
