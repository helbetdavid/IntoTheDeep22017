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
                extend.setTarget(1000);
                extend.setPower();
                if(extend.getPosition()>=990 && extend.getPosition()<=1010){
                    servoCam.trackTarget();
                    limeLight.logPipelineData();
                    tx = limeLight.getTargetTx();
                    ty = limeLight.getTargetTy();
                    // Align the robot laterally (strafe left/right) until tx is 0
                    double strafePower = -tx * 0.1; // Proportional control for tx
                    strafePower = Math.max(Math.min(strafePower, 1), -1); // Limit power to [-1, 1]

                    // Deadband: Stop if tx is close to 0
                    if (Math.abs(tx) < 1) {
                        strafePower = 0;
                    }

                    // Calculate motor powers for strafing
                    double frontLeftPower = strafePower;
                    double backLeftPower = -strafePower;
                    double frontRightPower = -strafePower;
                    double backRightPower = strafePower;

                    // Set motor powers
                    hwMap.leftFront.setPower(frontLeftPower);
                    hwMap.leftBack.setPower(backLeftPower);
                    hwMap.rightFront.setPower(frontRightPower);
                    hwMap.rightBack.setPower(backRightPower);
                    telemetry.update();
                }
                else if(currentGamepad1.b && !previousGamepad1.b){
                    extend.setTarget(0);
                    extend.setPower();
                    if(extend.getPosition()>=-10 && extend.getPosition()<=10){
                        servoCam.setAngle(0.5);
                        telemetry.update();
                    }

                }
            }
        }
    }

}
