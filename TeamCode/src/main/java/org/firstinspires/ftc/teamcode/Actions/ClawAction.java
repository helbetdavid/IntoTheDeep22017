package org.firstinspires.ftc.teamcode.Actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Claw;

public class ClawAction {
    private HwMap hwMap;
    private Claw claw;
    public ClawAction(HardwareMap hardwareMap){
        hwMap = new HwMap();
        hwMap.init(hardwareMap);

        claw = new Claw(hwMap.claw);
    }

    public class ClawOpen implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            claw.open();
            return false;
        }
    }

    public class ClawClose implements Action{
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            claw.close();
            return false;
        }
    }
    public Action clawOpen(){
        return new ClawOpen();
    }
    public Action clawClose(){
        return new ClawClose();
    }
}
