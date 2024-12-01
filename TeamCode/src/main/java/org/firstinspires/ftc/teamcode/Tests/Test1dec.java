package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.RR.TankDrive;
import org.firstinspires.ftc.teamcode.RR.tuning.TuningOpModes;

@Autonomous
public final class Test1dec extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(7.61, 61.2, 0);
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .turn(Math.toRadians(-90))
                        .strafeTo(new Vector2d(4.8, 33.3))
                .waitSeconds(3)
                        .setTangent(1)
                        .splineToSplineHeading(new Pose2d(54, 54, 44.78), Math.PI / 2)
//              .waitSeconds(3)
                        .turnTo(Math.toRadians(-90))
                        .strafeTo(new Vector2d(48.3, 40))
//              .waitSeconds(3)
                        .turnTo(Math.toRadians(44.78))
                        .strafeTo(new Vector2d(54, 54))
//              .waitSeconds(3)
                        .turnTo(Math.toRadians(-90))
                        .strafeTo(new Vector2d(58.5, 40))
//              .waitSeconds(3)
                        .turnTo(Math.toRadians(44.78))
                        .strafeTo(new Vector2d(54, 54))
//              .waitSeconds(3)
                        .turnTo(Math.toRadians(0))
                        .strafeTo(new Vector2d(55, 26))
//              .waitSeconds(3)
                        .turnTo(Math.toRadians(44.78))
                        .strafeTo(new Vector2d(54, 54))
//              .waitSeconds(3)
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
