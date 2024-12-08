package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Actions.ClawAct;
import org.firstinspires.ftc.teamcode.Actions.ClawRotateAct;
import org.firstinspires.ftc.teamcode.Actions.ExtendAct;
import org.firstinspires.ftc.teamcode.Actions.ServoCamAct;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystem.Claw;
import org.firstinspires.ftc.teamcode.SubSystem.Extend;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

@Autonomous
public final class test8dec extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(7.61, 61.2, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        ClawAct clawAct = new ClawAct(hardwareMap);
        ClawRotateAct clawRotateAct= new ClawRotateAct(hardwareMap);
        ServoCamAct servoCamAct= new ServoCamAct(hardwareMap);
        ExtendAct extendAct = new ExtendAct(hardwareMap,this.telemetry);


        Actions.runBlocking(
                new ParallelAction(
//                        clawAct.clawClose(),
//                        clawRotateAct.clawRotateUp(),
//                        servoCamAct.straight()
                        extendAct.setTarget(200),
                        extendAct.setPower()

                            
                )
        );
        waitForStart();

//        Actions.runBlocking(
//                drive.actionBuilder(beginPose)
//                        .strafeToLinearHeading(new Vector2d(4.8,30),Math.toRadians(-90))
//
//
//                        .setTangent(1)
//                        .splineToSplineHeading(new Pose2d(48, 48, -89.54), Math.PI / 2)
//                        .waitSeconds(1.5)
//                        .strafeToLinearHeading(new Vector2d(56,52),Math.toRadians(44.78))
//                        .waitSeconds(1.5)
//                        .strafeToLinearHeading(new Vector2d(58.5,45),Math.toRadians(-90))
//                        .waitSeconds(1.5)
//                        .strafeToLinearHeading(new Vector2d(56,52),Math.toRadians(44.78))
//                        .waitSeconds(1.5)
//                        .strafeToLinearHeading(new Vector2d(55,18),Math.toRadians(0))
//                        .waitSeconds(1.5)
//                        .strafeToLinearHeading(new Vector2d(56,52),Math.toRadians(44.78))
//                        .waitSeconds(1.5)
//                        .strafeTo(new Vector2d(33, 10))
//                        .turnTo(Math.toRadians(0))
//                        .waitSeconds(999999999)
//                        .turn(Math.toRadians(90))
//                        .build());
    }
}
