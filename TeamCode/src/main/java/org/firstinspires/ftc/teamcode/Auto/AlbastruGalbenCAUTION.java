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
import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.RR.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystem.ExtendNou;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;

@Autonomous
@Config
public final class AlbastruGalbenCAUTION extends LinearOpMode {
    public static double target = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(30.05, 63, Math.toRadians(0));
        HwMap hwMap = new HwMap();
        hwMap.init(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        ClawAction clawAction = new ClawAction(hardwareMap);
        ClawRotateAction clawRotateAction = new ClawRotateAction(hardwareMap);
        ServoCamAction servoCamAction = new ServoCamAction(hardwareMap);
        ExtendAction extendAction = new ExtendAction(hardwareMap, this.telemetry);
        LiftAction liftAction = new LiftAction(hardwareMap, this.telemetry);
        LimeLight limeLight = new LimeLight(hwMap.limelight, telemetry);
        ExtendNou extenderSubsystem = new ExtendNou(hwMap.extendo);



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
                        drive.actionBuilder(new Pose2d(30.05, 63, Math.toRadians(0)))
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
                        new SleepAction(0.3),
                        drive.actionBuilder(new Pose2d(51, 45, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(47.5, 41.8), Math.toRadians(-88))
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
                        drive.actionBuilder(new Pose2d(47.5, 41.8, Math.toRadians(-88)))
                                .strafeToLinearHeading(new Vector2d(51.5, 52.5), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(90),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.3),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0),
                        new SleepAction(0.5)
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(51.5, 52.5, Math.toRadians(45)))
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
                                .strafeToLinearHeading(new Vector2d(52.5, 51.5), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(90),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.3),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0),
                        new SleepAction(0.5)
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(52.5, 51.5, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(51, 45), Math.toRadians(45))
                                .build(),
                        liftAction.liftToPosition(60)

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(51, 45, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(55, 39), Math.toRadians(-45))
                                .build(),
                        clawAction.clawOpenAuto(),
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.auto(),
                        extendAction.extendToPosition(340),
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
                                .strafeToLinearHeading(new Vector2d(52.5, 51.5), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(90),
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


// aici ma joc eu hahhaha love david

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(52.5,51.5,Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(35, 8),Math.toRadians(180))
                                .build()

                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(35,8 ,Math.toRadians(180)))
                                .strafeTo(new Vector2d(30.5, 8))
                                .build()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.straight(),
                        clawAction.clawOpenSum(),
                        extendAction.extendToPosition(200),
                        liftAction.liftToPosition(750)

                )
        );


        double  xReal = (Math.tan(Math.toRadians(limeLight.getTargetTx())) * 23)*0.3937;
        double  yReal = ((Math.tan(Math.toRadians(limeLight.getTargetTy())) * 23)-2.7) * 11.76;


        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(52.5,51.5,Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(30.5+xReal, 8),Math.toRadians(180))
                                .build(),
                        extendAction.extendToPosition(extenderSubsystem.getCurrentPosition()+yReal)

                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.trackTarget(),
                        clawAction.clawOpenSum(),
                        new SleepAction(0.3),
                        liftAction.liftToPosition(0),
                        new SleepAction(0.3),
                        clawAction.clawClose(),
                        clawRotateAction.clawRotateInit(),
                        extendAction.extendToPosition(0)
                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(4450),
                        drive.actionBuilder(new Pose2d(30.5+xReal, 8, Math.toRadians(180)))
                                .strafeToLinearHeading(new Vector2d(51, 52), Math.toRadians(45))
                                .build()

                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(90),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.3),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0),
                        new SleepAction(0.5)
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(52.5,51.5,Math.toRadians(45)))
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
