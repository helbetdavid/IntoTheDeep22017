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
public final class AlbastruGalben extends LinearOpMode {
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(7.61, 62.8, Math.toRadians(-90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        ClawAction clawAction = new ClawAction(hardwareMap);
        ClawRotateAction clawRotateAction = new ClawRotateAction(hardwareMap);
        ServoCamAction servoCamAction = new ServoCamAction(hardwareMap);
        ExtendAction extendAction = new ExtendAction(hardwareMap, this.telemetry);
        LiftAction liftAction = new LiftAction(hardwareMap, this.telemetry);


        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawClose(),
                        clawRotateAction.clawRotateInit(),
                        servoCamAction.straight()

                )
        );
        waitForStart();

        Actions.runBlocking(
                        new ParallelAction(
                                liftAction.liftToPosition(1440),
                                drive.actionBuilder(beginPose)
                                        .strafeTo(new Vector2d(4.8, 34.5 ))
                                        .build()
                        )
        );
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(4.8,34,Math.toRadians(-90)))
                                        .strafeTo(new Vector2d(4.8,33))
                                        .build(),
                        extendAction.extendToPosition(350),

                        new SleepAction(1)
//                        clawAction.clawOpen(),
//                        new SleepAction(10000)
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawOpen(),
                        liftAction.liftToPosition(1000),
                        extendAction.extendToPosition(0)

                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(4.8,38,Math.toRadians(-90)))
                                .strafeTo(new Vector2d(50,46))
                                .build(),
                        liftAction.liftToPosition(0)

                ));

        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.straight(),
                        extendAction.extendToPosition(270),
                        new SleepAction(0.5),
                        clawAction.clawClose(),
                        new SleepAction(0.5),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)

                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(50,46,Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(54,50),Math.toRadians(44.78))

                                .build(),
                        liftAction.liftToPosition(4200)
                ));
        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(150),
                        clawRotateAction.clawRotateCollect(),
                        clawAction.clawOpen(),
                        extendAction.extendToPosition(0),
                        new SleepAction(0.3)


                )
        );

//        Actions.runBlocking(
//                new ParallelAction(
//                        drive.actionBuilder(new Pose2d(54,50,Math.toRadians(44.78)))
//                         .strafeToLinearHeading(new Vector2d(60.5,46),Math.toRadians(-90))
//                                .build(),
//                        liftAction.liftToPosition(0)
//
//                )
//        );
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        clawAction.clawOpen(),
//                        clawRotateAction.clawRotateDown(),
//                        servoCamAction.straight(),
//
////                        extendAction.extendToPosition(1000),
//                        new SleepAction(0.3),
//                        clawAction.clawClose(),
//                        clawRotateAction.clawRotateUp()
////                        extendAction.extendToPosition(0)
//                )
//        );
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        drive.actionBuilder(new Pose2d(60.5,46,Math.toRadians(-90)))
//                                .strafeToLinearHeading(new Vector2d(54,50),Math.toRadians(44.78))
//
//                                .build(),
//                        liftAction.liftToPosition(4450)
//
//                ));
//                Actions.runBlocking(
//                        new SequentialAction(
////                                extendAction.extendToPosition(700),
//                                clawAction.clawOpen()
////                                extendAction.extendToPosition(0)
//
//
//                        )
//                );
//        Actions.runBlocking(
//                new ParallelAction(
//                        drive.actionBuilder(new Pose2d(54,50,Math.toRadians(44.78)))
//                        .strafeToLinearHeading(new Vector2d(57,24),Math.toRadians(0))
//                                .build(),
//                        liftAction.liftToPosition(0)
//
//                )
//        );
//
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        clawRotateAction.clawRotateDown(),
//                        clawAction.clawOpen(),
////                        extendAction.extendToPosition(500),
//                        servoCamAction.lateral(),
//                        new SleepAction(1),
//                        clawAction.clawClose(),
////                        extendAction.extendToPosition(0),
//                        clawRotateAction.clawRotateUp()
//                        )
//        );
//        Actions.runBlocking(
//                new ParallelAction(
//                drive.actionBuilder(new Pose2d(57,24,Math.toRadians(0)))
//                        .strafeToLinearHeading(new Vector2d(54,50),Math.toRadians(44.78))
//                        .build(),
//                liftAction.liftToPosition(4450)
//        ));
//        Actions.runBlocking(
//                new SequentialAction(
////                        extendAction.extendToPosition(700),
//                        clawAction.clawOpen()
////                        extendAction.extendToPosition(0)
//
//
//                )
//        );
//        Actions.runBlocking(
//                new ParallelAction(
//                        drive.actionBuilder(new Pose2d(54,50,Math.toRadians(0)))
//                                .strafeToLinearHeading(new Vector2d(31, 8),Math.toRadians(0))
//                                .strafeTo(new Vector2d(28, 8))
//                                .build(),
//                        liftAction.liftToPosition(0)
//
//                ));
    }
}
