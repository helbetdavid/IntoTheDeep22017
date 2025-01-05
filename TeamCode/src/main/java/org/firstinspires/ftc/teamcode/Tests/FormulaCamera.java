package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystem.ClawRotate;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;
import org.firstinspires.ftc.teamcode.SubSystem.Lift;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

import java.util.ArrayList;
import java.util.List;

@TeleOp
@Config
public class FormulaCamera extends LinearOpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    private PIDController controller;
    public static double kP = 0.008, kI = 0.00001 , kD = 0.0001;
    public static double target = 0;

    public static double const1 =18.1;
//    public static double const2 = 28.175;
    public static double const2 = 0;
    double inches = 0;
    double ticks =0;
    double anglecam;
    ClawRotate clawRotate;

    @Override
    public void runOpMode() throws InterruptedException {

        controller = new PIDController(kP, kI, kD);

        HwMap hwMap = new HwMap();
        hwMap.init(hardwareMap);

        DcMotor extendo = hardwareMap.get(DcMotor.class, "extendo");
        Servo servoCam = hardwareMap.get(Servo.class,"servoCam");
        DcMotor perp = hardwareMap.get(DcMotor.class, "perp");
        Pose2d beginPose = new Pose2d(0, 0, 0);

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);



        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        Extend extend = new Extend(hwMap.extendo, telemetry);
        Lift lift = new Lift(hwMap.leftLift, hwMap.rightLift, telemetry);

        LimeLight limeLight = new LimeLight(hwMap.limelight, telemetry);


        double xCam, yCam, xReal, yReal;
        double finalTargetExt;
        double target1;
        double targetnouextend;


        while(opModeInInit() && !isStopRequested()) {
            lift.setTarget(750);
            lift.setPower();
        }

        clawRotate = new ClawRotate(hwMap.clawRotator);


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {


//            limeLight.logPipelineData();
            clawRotate.rotateDown();

            if(gamepad1.b){
                servoCam.setPosition(anglecam/180);
            }

        }
    }
}
