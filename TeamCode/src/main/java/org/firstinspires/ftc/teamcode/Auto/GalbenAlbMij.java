package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.InstantAction;
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
import org.firstinspires.ftc.teamcode.SubSystem.Lift;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

@Autonomous
@Config
public final class GalbenAlbMij extends LinearOpMode {
    public static double target = 0;

    double scan = 0;
    double xReal = 0;
    double yReal = 0;
    double currentExt = 0;
    double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(29, 62.5, Math.toRadians(0));
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ServoCam servoCam = new ServoCam(hwMap.servoCam, limeLight);
        Lift lift = new Lift(hwMap.leftLift, hwMap.rightLift, hwMap.rightFront, telemetry);


        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(() -> {
                            limeLight.setPipeline(1);
                        }),
                        clawAction.clawClose(),
                        new SleepAction(0.5),
                        clawRotateAction.clawRotateInit(),
                        servoCamAction.straight()
                )
        );
        sleep(50);
        waitForStart();

///lasat preload
        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(3200),
                        drive.actionBuilder(new Pose2d(29, 62.5, Math.toRadians(0)))
                                .strafeToLinearHeading(new Vector2d(51.5, 54.5), Math.toRadians(45))
                                .build()
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(60),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.1),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)
                )
        );

///lasat primu
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(51.5, 54.5, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(47.5, 41.8), Math.toRadians(-90))
                                .build(),
                        liftAction.liftToPosition(0)

                )
        );
///51,45
        Actions.runBlocking(
                new SequentialAction(
                        liftAction.liftToPosition(0),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.straight(),
                        extendAction.extendToPosition(195),
                        new SleepAction(0.2),
                        clawAction.clawClose(),
                        new SleepAction(0.2),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)

                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(3200),
                        drive.actionBuilder(new Pose2d(47.5, 41.8, Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(53, 53), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(45),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.1),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)

                )
        );
///lasat 2
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(53, 53, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(59, 41.8), Math.toRadians(-90))
                                .build(),
                        liftAction.liftToPosition(0)

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        liftAction.liftToPosition(0),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.straight(),
                        extendAction.extendToPosition(195),
                        new SleepAction(0.2),
                        clawAction.clawClose(),
                        new SleepAction(0.2),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)

                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(3200),
                        drive.actionBuilder(new Pose2d(59, 41.8, Math.toRadians(-90)))
                                .strafeToLinearHeading(new Vector2d(53, 53), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(45),
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
                        drive.actionBuilder(new Pose2d(53, 53, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(57, 38.5), Math.toRadians(-45))
                                .build(),
                        liftAction.liftToPosition(0)

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        liftAction.liftToPosition(0),
                        clawAction.clawOpenAuto(),
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.auto(),
                        extendAction.extendToPosition(335),
                        new SleepAction(0.2),
                        clawAction.clawClose(),
                        new SleepAction(0.2),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0),
                        servoCamAction.straight()
                )
        );
        Actions.runBlocking(
                new ParallelAction(
                        liftAction.liftToPosition(3200),
                        drive.actionBuilder(new Pose2d(57, 38.5, Math.toRadians(-45)))
                                .strafeToLinearHeading(new Vector2d(53, 53), Math.toRadians(45))
                                .build()

                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        extendAction.extendToPosition(45),
                        clawRotateAction.clawRotateDown(),
                        new SleepAction(0.1),
                        clawAction.clawOpen(),
                        clawRotateAction.clawRotateUp(),
                        extendAction.extendToPosition(0)
                )
        );


///STOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP


// aici ma joc eu hahhaha love david(talent only)

//        Actions.runBlocking(
//                new ParallelAction(
//                        drive.actionBuilder(new Pose2d(52.5, 53.5, Math.toRadians(45)))
//                                .setTangent(-2)
//                                .splineToLinearHeading(new Pose2d(25.5, 3, Math.toRadians(180)), Math.PI * 1.2)
//                                .build(),
//                        liftAction.liftToPosition(0)
//
//                )
//        );
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        clawRotateAction.clawRotateDown(),
//                        servoCamAction.straight(),
//                        clawAction.clawOpenSum(),
//                        extendAction.extendToPosition(220),
//                        liftAction.liftToPosition(750)
//                )
//        );
//
//
//        Actions.runBlocking(
//                new InstantAction(() -> {
//                    sleep(500);
//                    xReal = (Math.tan(Math.toRadians(limeLight.getTargetTx())) * 24) * 0.394;
//                    yReal = ((Math.tan(Math.toRadians(limeLight.getTargetTy())) * 24) - 2.6) * 11.76;
//                    servoCam.trackTarget();
//                })
//        );
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        drive.actionBuilder(new Pose2d(25.5, 3, Math.toRadians(180)))
//                                .strafeTo(new Vector2d(25.5, 3 + xReal))
//                                .build(),
//                        extendAction.extendToPosition(extenderSubsystem.getCurrentPosition() + yReal)
//                )
//        );
//
//        Actions.runBlocking(
//                new SequentialAction(
//                        clawAction.clawOpenSum(),
//                        new SleepAction(0.2),
//                        liftAction.liftToPosition(40),
//                        new SleepAction(0.4),
//                        clawAction.clawClose(),
//                        clawRotateAction.clawRotateInit(),
//                        extendAction.extendToPosition(0)
//                )
//        );
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        drive.actionBuilder(new Pose2d(25.5, 3 + xReal, Math.toRadians(180)))
//                                .setTangent(-0.2)
//                                .splineToLinearHeading(new Pose2d(51.5, 52.5, Math.toRadians(45)), Math.PI / 2)
//                                .build(),
//                        new SequentialAction(
//                                new SleepAction(1.5),
//                                liftAction.liftToPosition(3200)
//                        )
//                )
//        );
////        Actions.runBlocking(
////                new SequentialAction(
////                        drive.actionBuilder(new Pose2d(25, 3 + xReal, Math.toRadians(180)))
////                                .strafeToLinearHeading(new Vector2d(45, 8), Math.toRadians(180))
////                                .build()
////                )
////        );
////        Actions.runBlocking(
////                new ParallelAction(
////                        drive.actionBuilder(new Pose2d(45, 8, Math.toRadians(180)))
////                                .strafeToLinearHeading(new Vector2d(52.5, 53.5), Math.toRadians(45))
////                                .build(),
////                        liftAction.liftToPosition(4400)
////                )
////        );
//        Actions.runBlocking(
//                new SequentialAction(
//                        servoCamAction.straight(),
//                        extendAction.extendToPosition(105),
//                        clawRotateAction.clawRotateDown(),
//                        new SleepAction(0.1),
//                        clawAction.clawOpen(),
//                        clawRotateAction.clawRotateUp(),
//                        extendAction.extendToPosition(0),
//                        liftAction.liftToPosition(0)
//                )
//        );
    }
}
