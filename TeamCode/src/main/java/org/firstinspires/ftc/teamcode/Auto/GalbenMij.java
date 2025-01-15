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
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

@Autonomous
@Config
public final class GalbenMij extends LinearOpMode {
    public static double target = 0;

    double scan = 0;
    double xReal = 0;
    double yReal = 0;
    double currentExt = 0;
    double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(29, 63, Math.toRadians(0));
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


        Actions.runBlocking(
                new SequentialAction(
                        clawAction.clawClose(),
                        new SleepAction(0.5),
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
                        drive.actionBuilder(new Pose2d(47.5, 41.8, Math.toRadians(-90)))
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
                        extendAction.extendToPosition(0)

                )
        );
///lasat 2
        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(51.5, 54.5, Math.toRadians(45)))
                                .strafeToLinearHeading(new Vector2d(59, 41), Math.toRadians(-90))
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
                        drive.actionBuilder(new Pose2d(59, 41, Math.toRadians(-90)))
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



///STOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOP


// aici ma joc eu hahhaha love david(talent only)

        Actions.runBlocking(
                new ParallelAction(
                        drive.actionBuilder(new Pose2d(52.5,53.5,Math.toRadians(45)))
                                .setTangent(-2)
                                .splineToLinearHeading(new Pose2d(28, 3,Math.toRadians(180)),Math.PI*1.2)
                                .build(),
                        liftAction.liftToPosition(0)

                )
        );

        Actions.runBlocking(
                new ParallelAction(
                        clawRotateAction.clawRotateDown(),
                        servoCamAction.straight(),
                        clawAction.clawOpenSum(),
                        extendAction.extendToPosition(250),
                        liftAction.liftToPosition(750),
                        new SleepAction(2)
                )
        );


        Actions.runBlocking(
                new InstantAction(() -> {
                    xReal = (Math.tan(Math.toRadians(limeLight.getTargetTx())) * 23) * 0.394;
                    if (Math.signum(xReal) == 1.0)
                        xReal += 1;
                    else xReal -= 1;
                    yReal = ((Math.tan(Math.toRadians(limeLight.getTargetTy())) * 23) - 2.6) * 11.76;
                    telemetry.addData("xreal", xReal);
                    telemetry.addData("yreal", yReal);
                    telemetry.update();
                })
        );


        Actions.runBlocking(
                new InstantAction(() -> {
                    currentExt = extenderSubsystem.getCurrentPosition();
                    telemetry.addData("cure", currentExt);
                    telemetry.update();
                })
        );

        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(() -> {
                            angle = limeLight.getAngle();
                            telemetry.addData("Computed Angle", angle / 180);
                            telemetry.update();
                        })
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        drive.actionBuilder(new Pose2d(28, 3,Math.toRadians(180)))
                                .strafeTo(new Vector2d(28, 3 + xReal))
                                .build(),
                        extendAction.extendToPosition(currentExt + yReal)

                )
        );
        Actions.runBlocking(
                new SequentialAction(
                        new InstantAction(() -> {
                            servoCam.setAngle(angle / 180);
                        })
                )
        );

        Actions.runBlocking(
                new SequentialAction(
                        clawRotateAction.clawRotateDown(),
                        clawAction.clawOpenSum(),
                        new SleepAction(0.3),
                        liftAction.liftToPosition(0),
                        new SleepAction(0.3),
                        clawAction.clawClose(),
                        clawRotateAction.clawRotateInit(),
                        extendAction.extendToPosition(0)
                )
        );
//
//        Actions.runBlocking(
//                new ParallelAction(
//                        liftAction.liftToPosition(4400),
//                        drive.actionBuilder(new Pose2d(30.5+xReal, 8, Math.toRadians(180)))
//                                .strafeToLinearHeading(new Vector2d(51, 52), Math.toRadians(45))
//                                .build()
//
//                )
//        );
//        Actions.runBlocking(
//                new SequentialAction(
//                        extendAction.extendToPosition(90),
//                        clawRotateAction.clawRotateDown(),
//                        new SleepAction(0.3),
//                        clawAction.clawOpen(),
//                        clawRotateAction.clawRotateUp(),
//                        extendAction.extendToPosition(0),
//                        new SleepAction(0.5)
//                )
//        );
//        Actions.runBlocking(
//                new ParallelAction(
//                        drive.actionBuilder(new Pose2d(52.5,51.5,Math.toRadians(45)))
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
