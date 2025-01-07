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
public final class AlbastruAlbastru extends LinearOpMode {
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


//2200
//1500
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(-14, 62.8, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(-4.8, 36.5))
                                .build(),
                        liftAction.liftToPosition(2200)
                        )
        );

        Actions.runBlocking(
                new SequentialAction(
                        liftAction.liftToPosition(1400),
                        clawAction.clawOpen()
                        )
        );



        Actions.runBlocking(
                new SequentialAction(
                         new SleepAction(999999999)
                )
        );


        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(-5,37,Math.toRadians(-90)))
                                .strafeTo(new Vector2d(-5,30.5))
                                .build(),
                        extendAction.extendToPosition(280)
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawOpen(),
                        liftAction.liftToPosition(1300),
                        extendAction.extendToPosition(0)
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(-5,34,Math.toRadians(-90)))
                                .strafeTo(new Vector2d(-4.8, 36))
                                .strafeTo(new Vector2d(-31, 43))
                                .setTangent(60)
                                .splineToSplineHeading(new Pose2d(-50, 13, Math.toRadians(0)), Math.PI*1.21)
                                .strafeTo(new Vector2d(-50, 57))
                                .strafeToLinearHeading(new Vector2d(-56,13),Math.toRadians(0))
                                .strafeTo(new Vector2d(-60,13))
                                .strafeTo(new Vector2d(-60, 50))
                                .strafeToLinearHeading(new Vector2d(-61, 13), Math.toRadians(0))
                                .strafeTo(new Vector2d(-69,13))
                                .strafeTo(new Vector2d(-69, 50))
                                .strafeToLinearHeading(new Vector2d(-33, 55.5), Math.toRadians(180))

                                .build(),
                        liftAction.liftToPosition(0)
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        //roatate trebuie pe 0.7
                        clawRotateAction.clawRotateSpec(),
//                        new SleepAction(0.3),
                        clawAction.clawOpen(),
                        extendAction.extendToPosition(290),
                        new SleepAction(0.5 ),
                        clawAction.clawClose(),
                        extendAction.extendToPosition(0),
                        clawRotateAction.clawRotateUp()
                )
        );

                        Actions.runBlocking(
                                new ParallelAction(
                                        liftAction.liftToPosition(1450),
                                        drive.actionBuilder(new Pose2d(-33,55.5,Math.toRadians(180)))
                                                .strafeToLinearHeading(new Vector2d(-6.5, 34), Math.toRadians(-90))
                                                .build()
                                )
                        );

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(-5,37,Math.toRadians(-90)))
                                .strafeTo(new Vector2d(-6.5,30.5))
                                .build(),
                        extendAction.extendToPosition(280)
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawOpen(),
                        liftAction.liftToPosition(1300),
                        extendAction.extendToPosition(0)
                )
        );
        Actions.runBlocking(
                        new ParallelAction(
                                liftAction.liftToPosition(0),
                                drive.actionBuilder(new Pose2d(-6,34,Math.toRadians(180)))
                                        .strafeToLinearHeading(new Vector2d(-33, 57.5), Math.toRadians(180))
                                        .build()
                        )
                );
//                                .strafeToLinearHeading(new Vector2d(-8, 37), Math.toRadians(-90))
//                                .strafeToLinearHeading(new Vector2d(-33, 57.5), Math.toRadians(180))
//                                .strafeToLinearHeading(new Vector2d(-9.5, 37), Math.toRadians(-90))
//                                .strafeTo(new Vector2d(-49, 60))
//                                .build()
//
//

    }
}