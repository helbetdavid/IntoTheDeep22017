package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.ClawAct;
import org.firstinspires.ftc.teamcode.Actions.ClawRotateAct;
import org.firstinspires.ftc.teamcode.Actions.ExtendAct;
import org.firstinspires.ftc.teamcode.Actions.LiftAct;
import org.firstinspires.ftc.teamcode.Actions.ServoCamAct;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;

@Autonomous
@Config
public final class AlbGalb extends LinearOpMode {
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(7.61, 62.8, 0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        ClawAct clawAct = new ClawAct(hardwareMap);
        ClawRotateAct clawRotateAct = new ClawRotateAct(hardwareMap);
        ServoCamAct servoCamAct = new ServoCamAct(hardwareMap);
        ExtendAct extendAct = new ExtendAct(hardwareMap, this.telemetry);
        LiftAct liftAct = new LiftAct(hardwareMap, this.telemetry);


        Actions.runBlocking(
                new SequentialAction(
                        clawAct.clawClose(),
                        clawRotateAct.clawRotateInit(),
                        servoCamAct.straight()

                )
        );
        waitForStart();

        Actions.runBlocking(
                        new ParallelAction(
                                liftAct.liftToPosition(2300),
                                clawRotateAct.clawRotateUp(),
                                drive.actionBuilder(beginPose)
                                        .strafeToLinearHeading(new Vector2d(4.8, 38), Math.toRadians(-90))
                                        .build()
                        )
        );
        Actions.runBlocking(
            new SequentialAction(
                            liftAct.liftToPosition(1700),
                            new SleepAction(0.3), // Am modificat de la 0.5 la 0.3
                            clawAct.clawOpen()
            )
        );
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(4.8,38,Math.toRadians(-90)))
                                .strafeTo(new Vector2d(50,46))
                                .build(),
                        liftAct.liftToPosition(0)

                ));

        Actions.runBlocking(
                new SequentialAction(
                        clawAct.clawOpen(),
                        clawRotateAct.clawRotateDown(),
                        servoCamAct.straight(),
                        extendAct.extendToPosition(1000),
                        new SleepAction(0.5),
                        clawAct.clawClose(),
                        clawRotateAct.clawRotateUp(),
                        extendAct.extendToPosition(0)

                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(50,46,Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(54,50),Math.toRadians(44.78))

                                .build(),
                        liftAct.liftToPosition(4450)
                ));
        Actions.runBlocking(
                new SequentialAction(
                        extendAct.extendToPosition(700),
                        clawAct.clawOpen(),
                        extendAct.extendToPosition(0),
                        new SleepAction(0.3)


                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(54,50,Math.toRadians(44.78)))
                         .strafeToLinearHeading(new Vector2d(60.5,46),Math.toRadians(-90))
                                .build(),
                        liftAct.liftToPosition(0)

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        clawAct.clawOpen(),
                        clawRotateAct.clawRotateDown(),
                        servoCamAct.straight(),

                        extendAct.extendToPosition(1000),
                        new SleepAction(0.3),
                        clawAct.clawClose(),
                        clawRotateAct.clawRotateUp(),
                        extendAct.extendToPosition(0)
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(60.5,46,Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(54,50),Math.toRadians(44.78))

                                .build(),
                        liftAct.liftToPosition(4450)

                ));
                Actions.runBlocking(
                        new SequentialAction(
                                extendAct.extendToPosition(700),
                                clawAct.clawOpen(),
                                extendAct.extendToPosition(0)


                        )
                );
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(54,50,Math.toRadians(44.78)))
                        .strafeToLinearHeading(new Vector2d(57,24),Math.toRadians(0))
                                .build(),
                        liftAct.liftToPosition(0)

                )
        );


        Actions.runBlocking(
                new SequentialAction(
                        clawRotateAct.clawRotateDown(),
                        clawAct.clawOpen(),
                        extendAct.extendToPosition(500),
                        servoCamAct.lateral(),
                        new SleepAction(1),
                        clawAct.clawClose(),
                        extendAct.extendToPosition(0),
                        clawRotateAct.clawRotateUp()
                        )
        );
        Actions.runBlocking(
                new ParallelAction(
                drive.actionBuilder(new Pose2d(57,24,Math.toRadians(0)))
                        .strafeToLinearHeading(new Vector2d(54,50),Math.toRadians(44.78))
                        .build(),
                liftAct.liftToPosition(4450)
        ));
        Actions.runBlocking(
                new SequentialAction(
                        extendAct.extendToPosition(700),
                        clawAct.clawOpen(),
                        extendAct.extendToPosition(0)


                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(54,50,Math.toRadians(0)))
                                .strafeToLinearHeading(new Vector2d(31, 8),Math.toRadians(0))
                                .strafeTo(new Vector2d(28, 8))
                                .build(),
                        liftAct.liftToPosition(0)

                ));
    }
}
