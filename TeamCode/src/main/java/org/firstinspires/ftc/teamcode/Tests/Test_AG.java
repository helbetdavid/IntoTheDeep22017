package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

@Autonomous
public final class Test_AG extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(7.61, 61.2, 0);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
//                        .turn(Math.toRadians(-90))
//                        .strafeTo(new Vector2d(4.8, 30))
                        .strafeToLinearHeading(new Vector2d(4.8,30),Math.toRadians(-90))


                        .setTangent(1)
                        .splineToSplineHeading(new Pose2d(48, 48, -89.54), Math.PI / 2)
                .waitSeconds(1.5)
//                        .turnTo(Math.toRadians(-90))
//                        .strafeTo(new Vector2d(48, 48))



//                        .turnTo(Math.toRadians(44.78))
//                        .strafeTo(new Vector2d(54, 54))
                        .strafeToLinearHeading(new Vector2d(56,52),Math.toRadians(44.78))
                .waitSeconds(1.5)
//                        .turnTo(Math.toRadians(-90))
//                        .strafeTo(new Vector2d(58.5,48))
                        .strafeToLinearHeading(new Vector2d(58.5,45),Math.toRadians(-90))
                .waitSeconds(1.5)
//                        .turnTo(Math.toRadians(44.78))
//                        .strafeTo(new Vector2d(54, 54))
                        .strafeToLinearHeading(new Vector2d(56,52),Math.toRadians(44.78))
                .waitSeconds(1.5)
//                        .turnTo(Math.toRadians(0))
//                        .strafeTo(new Vector2d(55, 20))
                        .strafeToLinearHeading(new Vector2d(55,18),Math.toRadians(0))
                .waitSeconds(1.5)
//                        .turnTo(Math.toRadians(44.78))
//                        .strafeTo(new Vector2d(54, 54))
                        .strafeToLinearHeading(new Vector2d(56,52),Math.toRadians(44.78))
                .waitSeconds(1.5)
                        .strafeTo(new Vector2d(33, 10))
                        .turnTo(Math.toRadians(0))
//              .lineToX(61.2)
//              .turn(Math.toRadians(90))
//              .lineToY(30)
//              .turn(Math.toRadians(90))
//              .lineToX(0)
//              .turn(Math.toRadians(90))
//              .lineToY(0)
                        .waitSeconds(999999999)
                        .turn(Math.toRadians(90))
                        .build());
        }
    }
