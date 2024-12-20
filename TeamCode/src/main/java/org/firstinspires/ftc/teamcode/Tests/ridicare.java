package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.ExtendNou;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;

@TeleOp
@Config
public class ridicare extends LinearOpMode {
    public static double targetlift=0;
    public static double targetExt=0;
    private HwMap hwMap;

    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HwMap();
        hwMap.init(hardwareMap);
        ExtendNou extenderSubsystem = new ExtendNou(hwMap.extendo);
        Lift lift = new Lift(hwMap.leftLift, hwMap.rightLift, telemetry);

        waitForStart();
        while(opModeIsActive()){
            extenderSubsystem.updatePID();

            if(gamepad1.a){
                targetlift=2000;
                targetExt=170;
            }
            if(gamepad1.b){
                targetlift=0;
                targetExt=0;
            }
            if(gamepad1.y){
                hwMap.extendo.setPower(-1);
            }
//1            lift.setTarget(targetlift);
//            extenderSubsystem.runToTarget(targetExt);
//            lift.setPower();
        }
    }
}
