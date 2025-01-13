package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Actions.ClawAction;
import org.firstinspires.ftc.teamcode.Actions.ClawRotateAction;
import org.firstinspires.ftc.teamcode.Actions.ExtendAction;
import org.firstinspires.ftc.teamcode.Actions.LiftAction;
import org.firstinspires.ftc.teamcode.Actions.ServoCamAction;
import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;

@TeleOp
@Config
public class Luatdepegardusor extends LinearOpMode {

    private PIDController controllercam;
    private LimeLight limeLight;
    private HwMap hwMap;


    public static double kPcam = 0.02, kIcam=0.00025, kDcam=0.0007;
    public static double targetcam=34;

    @Override
    public void runOpMode() throws InterruptedException {

        controllercam = new PIDController(kPcam, kIcam, kDcam);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        DcMotor extendo = hardwareMap.get(DcMotor.class, "extendo");
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);


        hwMap = new HwMap();
        hwMap.init(hardwareMap);
        limeLight = new LimeLight(hwMap.limelight, telemetry);
        limeLight.setPipeline(0);

        waitForStart();

        while (opModeIsActive()) {



                double targetArea = limeLight.getTargetArea(); // Initialize target area
                controllercam.setPID(kPcam, kIcam, kDcam);

                double calculcam = controllercam.calculate(targetArea, targetcam);

                extendo.setPower(calculcam);






            telemetry.addData("Target Area", targetcam);
            telemetry.addData("Current Area",targetArea ); // Show the current target area
            telemetry.update();
        }
    }
}