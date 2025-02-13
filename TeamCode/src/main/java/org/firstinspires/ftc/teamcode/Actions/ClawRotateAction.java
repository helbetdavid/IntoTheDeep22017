package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.ClawRotate;

public class ClawRotateAction {
    private HwMap hwMap;
    private ClawRotate clawRotate;
    public ClawRotateAction(HardwareMap hardwareMap){
        hwMap = new HwMap();
        hwMap.init(hardwareMap);

        clawRotate = new ClawRotate(hwMap.clawRotator);
    }

    public class ClawRotateUp implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            clawRotate.rotateUp();
            return false;
        }
    }

    public class ClawRotateDown implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            clawRotate.rotateDown();
            return false;
        }
    }
    public class ClawRotateBasket implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            clawRotate.rotateBasket();
            return false;
        }
    }
    public class ClawRotateSub implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            clawRotate.rotateSub();
            return false;
        }
    }

    public Action clawRotateInit(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawRotate.rotateInit();
                return false;
            }
        };
    }

    public Action clawRotateCollect(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawRotate.rotateBasket();
                return false;
            }
        };
    }
    public class ClawRotateBasketFull implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            clawRotate.rotateBasket();
            return false;
        }
    }



    public Action clawRotateSpec(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawRotate.rotateSpec();
                return false;
            }
        };
    }


    public Action  clawRotateUp(){
        return new ClawRotateUp();
    }
    public Action clawRotateDown(){
        return new ClawRotateDown();
    }
    public Action clawRotateSub(){
        return new ClawRotateSub();
    }
    public Action clawRotateBasket(){
        return new ClawRotateBasket();
    }
    public Action clawRotateBasketFull(){
        return new ClawRotateBasketFull();
    }

}
