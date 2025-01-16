package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.ClawAction;
import org.firstinspires.ftc.teamcode.Actions.ClawRotateAction;
import org.firstinspires.ftc.teamcode.Actions.ExtendAction;
import org.firstinspires.ftc.teamcode.Actions.LiftAction;
import org.firstinspires.ftc.teamcode.Actions.ServoCamAction;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

@Autonomous
@Config
public final class Colorate extends LinearOpMode {
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-14, 62.8, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        ClawAction clawAction = new ClawAction(hardwareMap);
        ClawRotateAction clawRotateAction = new ClawRotateAction(hardwareMap);
        ServoCamAction servoCamAction = new ServoCamAction(hardwareMap);
        ExtendAction extendAction = new ExtendAction(hardwareMap, this.telemetry);
        LiftAction liftAction = new LiftAction(hardwareMap, this.telemetry);


        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawClose(),
                        clawRotateAction.clawRotateSub(),
                        servoCamAction.straight()
                )
        );
        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        clawAction.clawClose(),
                        clawRotateAction.clawRotateUp(),
                        servoCamAction.straight(),
                        drive.actionBuilder(new Pose2d(-14, 62.8, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(2.7, 36))
                                .build(),
                        liftAction.liftToPosition(1460)
                )
        );
///-2.3
///0.2
///2.7
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(2.7, 36, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(2.7, 33))
                                .build(),
                        extendAction.extendToPosition(320)
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawOpen(),
                        liftAction.liftToPosition(1360),
                        extendAction.extendToPosition(0),
                        liftAction.liftToPosition(0)
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(2.7, 33, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(-31, 43))
                                .setTangent(60)
                                .splineToSplineHeading(new Pose2d(-50, 12, Math.toRadians(0)), Math.PI*1.15)
                                .strafeTo(new Vector2d(-50, 55))
                                .strafeToLinearHeading(new Vector2d(-56,12),Math.toRadians(0))
                                .strafeTo(new Vector2d(-61,12))
                                .strafeTo(new Vector2d(-61, 50))
                                .strafeToLinearHeading(new Vector2d(-41.5, 50), Math.toRadians(91))
                                .build()
                )
        );
///puspus

        Actions.runBlocking(
                new SequentialAction(
                        clawRotateAction.clawRotateBasket(),
                        servoCamAction.straight(),
                        new SleepAction(0.7)
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        extendAction.extendToPosition(127),
                        liftAction.liftToPosition(330)
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawClose(),
                        new SleepAction(0.3),
                        liftAction.liftToPosition(700),
                        new SleepAction(0.3),
                        extendAction.extendToPosition(0),
                        drive.actionBuilder(new Pose2d(-41.5, 50, Math.toRadians(91)))
                                .turnTo(Math.toRadians(-90))
                                .build(),
                        liftAction.liftToPosition(0)
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        clawRotateAction.clawRotateUp(),
                        drive.actionBuilder(new Pose2d(-41.5, 50, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(0.2, 36))
                                .build(),
                        liftAction.liftToPosition(1460)
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(0.2, 36, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(0.2, 33))
                                .build(),
                        extendAction.extendToPosition(310)
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawOpen(),
                        liftAction.liftToPosition(1360),
                        extendAction.extendToPosition(0),
                        drive.actionBuilder(new Pose2d(0.2, 33, Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(0.2, 34), Math.toRadians(-90))
                                .build(),
                        liftAction.liftToPosition(0),
                        clawRotateAction.clawRotateUp(),
                        drive.actionBuilder(new Pose2d(0.2, 33, Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(-46.5, 52), Math.toRadians(91))
                                .build()
                )
        );

///pus de ala

///pus ala 2
    }
}