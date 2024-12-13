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
public final class RosuGalben extends LinearOpMode {
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-14, 62.3, 0);
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
                        drive.actionBuilder(new Pose2d(4.8, 38, Math.toRadians(-90)))
                                .strafeTo(new Vector2d(50, 46))
                                .build(),
                        liftAct.liftToPosition(0)

                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(50, 46, Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(-3.8, 33.3), -89.54)
                                .strafeTo(new Vector2d(-31, 43))
                                .setTangent(60)
                                .splineToSplineHeading(new Pose2d(-46, 13, 89.54), Math.PI)
                                .strafeTo(new Vector2d(-45, 55))
                                .setTangent(-1)
                                .splineToLinearHeading(new Pose2d(-58, 13, 89.54), Math.PI)
                                .strafeTo(new Vector2d(-58, 50))
                                .setTangent(-1)
                                .splineToLinearHeading(new Pose2d(-60, 13, 89.54), Math.PI)
                                .strafeTo(new Vector2d(-60, 50))
                                .strafeTo(new Vector2d(-37, 50))
                                .strafeTo(new Vector2d(-37, 55))
                                .build()
                )
        );

    }
}
