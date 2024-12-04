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
public final class Test_AA extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-16.479, 61.2, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        //                        .turn(Math.toRadians(-90))
//                        .strafeTo(new Vector2d(-3.8, 33.3))
                        .strafeToLinearHeading(new Vector2d(-3.8, 33.3), -89.54)
//              .waitSeconds(3)
                        .setTangent(1)
                        .splineToSplineHeading(new Pose2d(-41, 56, 89.54), Math.PI / 2)
//              .waitSeconds(3)
                        //.setTangent(0)
                        //.splineToConstantHeading(new Vector2d(  -36, 10), Math.PI/2.15)

                        .strafeTo(new Vector2d(-35, 10))
                        .strafeTo(new Vector2d(-46, 13))
                        .strafeTo(new Vector2d(-46, 54))
//                  .waitSeconds(3)
                        //.setTangent(1)
                        //.splineToSplineHeading(new Pose2d(-3.8, 33.3,-90), Math.PI/ 2)
                        .strafeToLinearHeading(new Vector2d(-3.8, 33.3), -89.54)
                        .setTangent(1)
                        .splineToSplineHeading(new Pose2d(-48, 54, 89.54), Math.PI / 2)
                        .strafeToLinearHeading(new Vector2d(-3.8, 33.3), -89.54)
                        .strafeTo(new Vector2d(-54, 59))

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