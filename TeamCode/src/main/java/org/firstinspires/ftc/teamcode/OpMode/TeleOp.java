package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.ClawAct;
import org.firstinspires.ftc.teamcode.Actions.ClawRotateAct;
import org.firstinspires.ftc.teamcode.Actions.ExtendAct;
import org.firstinspires.ftc.teamcode.Actions.LiftAct;
import org.firstinspires.ftc.teamcode.Actions.ServoCamAct;
import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;


import java.util.ArrayList;
import java.util.List;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    public enum RoboState {
        Neutral,
        CollectingSum,
        RetractCollectingSum,
        CollectingGate,
        RetractCollectingGate,
        ScoringSum,
        RetractScoringSum,
        ScoringBasket,
        RetractScoringBasket
    }
    private HwMap hwMap;
    RoboState roboState = RoboState.Neutral;
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
        LimeLight limeLight = new LimeLight(hwMap.limelight, this.telemetry);

        Actions.runBlocking(
                new SequentialAction(
                        clawAct.clawClose(),
                        clawRotateAct.clawRotateInit(),
                        servoCamAct.straight()
                )
        );
        waitForStart();
        while(opModeIsActive() && !isStopRequested()){
            TelemetryPacket packet = new TelemetryPacket();
            switch (roboState){
                case Neutral:
                    runningActions.add(new SequentialAction(
                            clawAct.clawOpen(),
                            clawRotateAct.clawRotateInit(),
                            servoCamAct.straight(),
                            liftAct.liftToPosition(0),
                            extendAct.extendToPosition(0)
                    ));
                    if(gamepad2.a){
                        roboState = RoboState.CollectingSum;
                    } else if (gamepad2.b) {
                        roboState = RoboState.CollectingGate;
                    } else if (gamepad2.x) {
                        roboState = RoboState.ScoringSum;
                    } else if (gamepad2.y) {
                        roboState = RoboState.ScoringBasket;
                    }
                    break;
                case CollectingSum:
                    double tx = limeLight.getTargetTx();
                    double ty = limeLight.getTargetTy();

                    runningActions.add(new SequentialAction(
                            clawAct.clawOpen(),
                            liftAct.liftToPosition(200),
                            extendAct.extendToPosition(1200),
                            clawRotateAct.clawRotateDown(),
                            servoCamAct.trackTarget()

                    ));
                    if(gamepad2.dpad_down){
                        roboState = RoboState.RetractCollectingSum;
                    }
                    break;

                case RetractCollectingSum:
                    runningActions.add(
                            new SequentialAction(
                                    liftAct.liftToPosition(0),

                                    clawAct.clawClose(),
                                    servoCamAct.straight(),
                                    clawRotateAct.clawRotateInit(),
                                    extendAct.extendToPosition(0)
                            )
                    );
                      if (gamepad2.x) {
                        roboState = RoboState.ScoringSum;}
                      else if (gamepad2.y) {
                        roboState = RoboState.ScoringBasket;
                      }
                    break;

                case CollectingGate:
                    if(gamepad2.dpad_down){
                        roboState = RoboState.RetractCollectingGate;
                    }

                    break;
                case RetractCollectingGate:
                    // retract
                    if (gamepad2.x) {
                        roboState = RoboState.ScoringSum;}
                    else if (gamepad2.y) {
                        roboState = RoboState.ScoringBasket;
                    }
                    break;
                case ScoringSum:
                    runningActions.add(
                            new SequentialAction(
                                    clawAct.clawClose(),
                                    clawRotateAct.clawRotateUp(),
                                    servoCamAct.straight(),
                                    liftAct.liftToPosition(2300)
                            )
                    );
                    if(gamepad2.start){
                        roboState = RoboState.RetractScoringSum;

                    }
                    break;
                case RetractScoringSum:
                    runningActions.add(
                            new SequentialAction(
                                    liftAct.liftToPosition(1700),
                                    new SleepAction(0.3),
                                    clawAct.clawOpen()

                            )
                    );
                    roboState = RoboState.Neutral;
                    break;
                case ScoringBasket:
                    runningActions.add(
                            new SequentialAction(

                                    liftAct.liftToPosition(4450),
                                    extendAct.extendToPosition(300),

                                    clawAct.clawOpen()
                                    )
                    );
                    if(gamepad2.dpad_down){
                        roboState = RoboState.RetractScoringBasket;
                    }
                    break;
                case RetractScoringBasket:
                    runningActions.add(
                            new SequentialAction(
                                    extendAct.extendToPosition(0),
                                    liftAct.liftToPosition(0)
                            )
                    );
                    roboState = RoboState.Neutral;
                    break;
            }

            telemetry.addData("State", roboState);
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
