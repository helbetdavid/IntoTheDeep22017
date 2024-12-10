package org.firstinspires.ftc.teamcode.OpMode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Actions.ClawAct;
import org.firstinspires.ftc.teamcode.Actions.ClawRotateAct;
import org.firstinspires.ftc.teamcode.Actions.ExtendAct;
import org.firstinspires.ftc.teamcode.Actions.LiftAct;
import org.firstinspires.ftc.teamcode.Actions.ServoCamAct;
import org.firstinspires.ftc.teamcode.HwMap;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class manual extends LinearOpMode {

    public enum RoboState {
        Start,
        CollectingSum,
        RetractSumCollecting,
        CollectingGate,
        RetractGateCollecting,
        ScoringSum,
        RetractSumScoring,
        ScoringBasket,
        RetractBasketScoring
    };
    private HwMap hwMap;

    RoboState roboState = RoboState.Start;



    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    @Override
    public void runOpMode() throws InterruptedException {
        hwMap = new HwMap();
        hwMap.init(hardwareMap);
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
        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            // Mecanum drive
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;


            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            hwMap.leftFront.setPower(frontLeftPower);
            hwMap.leftBack.setPower(backLeftPower);
            hwMap.rightFront.setPower(frontRightPower);
            hwMap.rightBack.setPower(backRightPower);

            // updated based on gamepads

//            } else if (gamepad2.a) {
//                roboState = RoboState.CollectingSum;
//
//            }


            switch (roboState){
                case Start:
                    runningActions.add(
                            new SequentialAction(
                                    clawAct.clawClose(),
                                    clawRotateAct.clawRotateInit(),
                                    servoCamAct.straight(),
                                    liftAct.liftToPosition(0),
                                    extendAct.extendToPosition(0)
                            )
                    );
                if(gamepad2.start){
                    roboState = RoboState.Start;
                }
                else if(gamepad2.b){
                    roboState = RoboState.CollectingGate;
                }
                else if(gamepad2.y){
                    roboState = RoboState.ScoringSum;
                }
                else if(gamepad2.x) {
                    roboState = RoboState.ScoringBasket;
                }
                    break;
                case CollectingSum:
                    break;
                case RetractSumCollecting:
                    break;
                case CollectingGate:
                    break;
                case RetractGateCollecting:
                    break;
                case ScoringSum:
                    Actions.runBlocking(
                            new SequentialAction(
                                    clawAct.clawClose(),
                                    clawRotateAct.clawRotateUp(),
                                    servoCamAct.straight(),
                                    liftAct.liftToPosition(2300)
                            )
                    );
                    if (gamepad2.a) {
                        roboState = RoboState.RetractSumScoring;
                    }
                    break;
                case RetractSumScoring:
                    Actions.runBlocking(
                            new SequentialAction(
                                    liftAct.liftToPosition(0),
                                    clawAct.clawOpen()
                            )
                    );
                    roboState = RoboState.Start;
                    break;
            }
            telemetry.addData("Current State", roboState);
            telemetry.update();

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            dash.sendTelemetryPacket(packet);
        }
    }
}
