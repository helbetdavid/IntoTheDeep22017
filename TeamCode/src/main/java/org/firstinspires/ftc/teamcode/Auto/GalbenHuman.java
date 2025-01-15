package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
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
public final class GalbenHuman extends LinearOpMode {
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


//
//
        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(4400),
                        drive.actionBuilder(new Pose2d(29, 63, Math.toRadians(0)))
                                .strafeToLinearHeading(new Vector2d(51.5, 54.5), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(70),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.2),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0),
                        liftAction.liftToPosition(0)

                )
        );

///lasat primu


        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(51.5, 54.5, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(-30, 60), Math.toRadians(-180))
                                .build()


                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.straight(),
                        extendAction.extendToPosition(250),
                        new SleepAction(0.2),
                        clawAction.clawClose(),
                        new SleepAction(0.2),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)

                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(-30, 60, Math.toRadians(-180)))
                                .strafeToLinearHeading(new Vector2d(53.5, 57.5), Math.toRadians(45))
                                .build()

                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        liftAction.liftToPosition(4400),
                        extendAction.extendToPosition(70),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.2),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0),
                        liftAction.liftToPosition(0)

                )
        );

///lasat human

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(53.5, 57.5, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(48, 46.8), Math.toRadians(-90))
                                .build(),
                        liftAction.liftToPosition(0)

                )
        );
///51,45
        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.straight(),
                        extendAction.extendToPosition(190),
                        new SleepAction(0.2),
                        clawAction.clawClose(),
                        new SleepAction(0.2),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)

                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(4400),
                        drive.actionBuilder(new Pose2d(48, 46.8, Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(53.5, 57.5), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(70),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.2),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)

                )
        );
///lasat 2
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(53.5, 57.5, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(59.5, 46), Math.toRadians(-90))
                                .build(),
                        liftAction.liftToPosition(0)

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.straight(),
                        extendAction.extendToPosition(190),
                        new SleepAction(0.2),
                        clawAction.clawClose(),
                        new SleepAction(0.2),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)

                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(4400),
                        drive.actionBuilder(new Pose2d(59.5, 46, Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(52.5, 53.5), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(70),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.1),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)
                )
        );
///lasa 3
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(52.5, 53.5, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(57, 38.5), Math.toRadians(-45))
                                .build(),
                        liftAction.liftToPosition(60)

                )
        );

        Actions.runBlocking(
                new SequentialAction(

                        clawAction.clawOpenAuto(),
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.auto(),
                        extendAction.extendToPosition(350),
                        new SleepAction(0.3),
                        clawAction.clawClose(),
                        new SleepAction(0.15),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0),
                        servoCamAction.straight()

                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(4400),
                        drive.actionBuilder(new Pose2d(57, 38.5, Math.toRadians(-45)))
                                .strafeToLinearHeading(new Vector2d(52.5, 53.5), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(70),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.2),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)
                )
        );

///lasat 4




///STOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP



        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(53.5,53.5,Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(35, 8),Math.toRadians(180))
                                .build(),
                        liftAction.liftToPosition(1430),
                        clawRotateAction.clawRotateCollect()


                ));
        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(35,8 ,Math.toRadians(180)))
                                .strafeTo(new Vector2d(30.5, 8))
                                .build(),
                        new SleepAction(10)
                )
        );



    }
}




