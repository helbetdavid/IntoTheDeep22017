package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

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
    double tx=0;
    double ty=0;
    @Override
    public void runOpMode() {
        // Initialize hardware and telemetry
        hwMap = new HwMap();
        hwMap.init(hardwareMap);


        // Create the Extend subsystem
        extend = new Extend(hwMap.extendo, telemetry);

        limeLight = new LimeLight(hwMap.limelight, telemetry);

        servoCam = new ServoCam(hwMap.servoCam,limeLight);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();


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
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);
            if(currentGamepad1.a && !previousGamepad1.a){
                extend.setTarget(target);
                extend.setPower();
                if(extend.getPosition()>target-5 && extend.getPosition()<target+5){
                    servoCam.trackTarget();
                }
            }
            else if(currentGamepad1.b && !previousGamepad1.b){
                    servoCam.setAngle(0.5);
                }
            }
            limeLight.logPipelineData();
            telemetry.update();
        }
    }

