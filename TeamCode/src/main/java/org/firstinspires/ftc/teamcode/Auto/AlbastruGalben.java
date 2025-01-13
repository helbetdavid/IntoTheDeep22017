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
        Pose2d beginPose = new Pose2d(29, 63, Math.toRadians(0));
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
        sleep(50);
        waitForStart();


        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(4450),
                        drive.actionBuilder(new Pose2d(29, 63, Math.toRadians(0)))
                                .strafeToLinearHeading(new Vector2d(51.5, 54.5), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(70),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.3),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0),
                        new SleepAction(0.3)
                )
        );

///lasat primu
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(51.5, 54.5, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(47.5, 41.8), Math.toRadians(45))
                                .build(),
                        liftAction.liftToPosition(0)

                )
        );
///51,45
        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(0.3),
                        drive.actionBuilder(new Pose2d(47.5, 41.8, Math.toRadians(45)))
//                                .strafeToLinearHeading(new Vector2d(47.5, 41.8), Math.toRadians(-90))
                                .turnTo(Math.toRadians(-90))
                                .build(),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.straight(),
                        extendAction.extendToPosition(170),
                        new SleepAction(0.3),
                        clawAction.clawClose(),
                        new SleepAction(0.3),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)

                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(4450),
                        drive.actionBuilder(new Pose2d(47.5, 41.8, Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(51.5, 54.5), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(70),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.3),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0),
                        new SleepAction(0.5)
                )
        );
///lasat 2
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(51.5, 54.5, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(51, 45), Math.toRadians(45))
                                .build(),
                        liftAction.liftToPosition(0)

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(51, 45, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(59, 41.5), Math.toRadians(-90))
                                .build(),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.straight(),
                        extendAction.extendToPosition(170),
                        new SleepAction(0.3),
                        clawAction.clawClose(),
                        new SleepAction(0.3),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)

                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(4450),
                        drive.actionBuilder(new Pose2d(59, 41.5, Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(52.5, 53.5), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(70),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.3),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0),
                        new SleepAction(0.5)
                )
        );
///lasa 3
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(52.5, 53.5, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(51, 45), Math.toRadians(45))
                                .build(),
                        liftAction.liftToPosition(60)

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(51, 45, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(55, 40), Math.toRadians(-45))
                                .build(),
                        clawAction.clawOpenAuto(),
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.auto(),
                        extendAction.extendToPosition(340),
                        new SleepAction(0.3),
                        clawAction.clawClose(),
                        new SleepAction(0.3),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0),
                        servoCamAction.straight()

                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(4450),
                        drive.actionBuilder(new Pose2d(59, 41.5, Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(52.5, 53.5), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(70),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.3),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0),
                        new SleepAction(0.5),
                        liftAction.liftToPosition(0)
                )
        );




///STOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP


//
//        Actions.runBlocking(
//                new ParallelAction(
//                        drive.actionBuilder(new Pose2d(53.5,53.5,Math.toRadians(45)))
//                                .strafeToLinearHeading(new Vector2d(35, 8),Math.toRadians(180))
//                                .build(),
//                        liftAction.liftToPosition(1430),
//                        clawRotateAction.clawRotateCollect()
//
//
//                ));
//        Actions.runBlocking(
//                new SequentialAction(
//                        drive.actionBuilder(new Pose2d(35,8 ,Math.toRadians(180)))
//                                .strafeTo(new Vector2d(30.5, 8))
//                                .build(),
//                        new SleepAction(10)
//                )
//        );



    }
}




