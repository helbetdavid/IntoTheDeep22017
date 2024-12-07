package org.firstinspires.ftc.teamcode.Actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.ClawRotate;

public class ClawRotateAct {
    private HwMap hwMap;
    private ClawRotate clawRotate;
    public ClawRotateAct(HardwareMap hardwareMap){
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
    public Action clawRotateInit(){
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                clawRotate.rotateInit();
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
    public Action clawTotateInit(){
        return clawRotateInit();
    }

}
