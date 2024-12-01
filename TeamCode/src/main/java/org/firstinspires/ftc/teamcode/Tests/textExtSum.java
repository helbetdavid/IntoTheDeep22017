package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.RR.TankDrive;
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
    @Override
    public void runOpMode() {
        // Initialize hardware and telemetry
        hwMap = new HwMap();
        hwMap.init(hardwareMap);


        // Create the Extend subsystem
        extend = new Extend(hwMap.extendo, telemetry);

        limeLight = new LimeLight(hwMap.limelight, telemetry);

        servoCam = new ServoCam(hwMap.servoCam,limeLight);


        // Set a target position

        waitForStart();

        while (opModeIsActive()) {
//            // Continuously update motor power
//            extend.setTarget(target);
//            extend.setPower();
//
//            limeLight.logPipelineData();
//
//            if(gamepad1.a){
//                servoCam.trackTarget();  // This will adjust the servo based on LimeLight's angle
//            }
//            else if(gamepad1.b){
//                servoCam.setAngle(0.5);  // This will set the servo to a specific angle
//            }
//            telemetry.addData("Target Position", target);
//            telemetry.addData("Current Position", hwMap.extendo.getCurrentPosition());
//            telemetry.update();
            if(gamepad1.a){
                extend.setTarget(1000);
                extend.setPower();
                sleep(1000);

                limeLight.logPipelineData();
                servoCam.trackTarget();

            }
            else if(gamepad1.b){
                extend.setTarget(0);
                extend.setPower();
                limeLight.logPipelineData();
                servoCam.setAngle(0.5);
            }
        }
    }

}
