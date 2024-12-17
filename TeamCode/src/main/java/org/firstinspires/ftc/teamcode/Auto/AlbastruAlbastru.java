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
public final class AlbastruAlbastru extends LinearOpMode {
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-14, 62.8, 0);
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
                        liftAction.liftToPosition(2300),
                        clawRotateAction.clawRotateUp(),
                        drive.actionBuilder(beginPose)
                                .strafeToLinearHeading(new Vector2d(4.8, 38), Math.toRadians(-90))
                                .build()
                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        liftAction.liftToPosition(1700),
                        new SleepAction(0.3), // Am modificat de la 0.5 la 0.3
                        clawAction.clawOpen()
                )
        );

        Actions.runBlocking(new SequentialAction(
                drive.actionBuilder(new Pose2d(4.8, 38, Math.toRadians(-90)))
                        .strafeTo(new Vector2d(-42, 57))
                        .build(),
                liftAction.liftToPosition(0)
        ));

    }
}