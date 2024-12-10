package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.ClawAct;
import org.firstinspires.ftc.teamcode.Actions.ClawRotateAct;
import org.firstinspires.ftc.teamcode.Actions.ExtendAct;
import org.firstinspires.ftc.teamcode.Actions.LiftAct;
import org.firstinspires.ftc.teamcode.Actions.ServoCamAct;
import org.firstinspires.ftc.teamcode.HwMap;

import java.util.ArrayList;
import java.util.List;

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

        Actions.runBlocking(
                new SequentialAction(
                        clawAct.clawClose(),
                        clawRotateAct.clawRotateInit(),
                        servoCamAct.straight()
                )
        );
        waitForStart();
        while(opModeIsActive()){
            switch (roboState){
                case Neutral:

                    if(gamepad1.a){
                        roboState = RoboState.CollectingSum;
                    }
                    break;
            }
        }
    }
}
