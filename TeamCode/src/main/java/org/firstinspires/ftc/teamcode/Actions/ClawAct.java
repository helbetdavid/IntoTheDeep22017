package org.firstinspires.ftc.teamcode.Actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.Claw;

public class ClawAct {
    private HwMap hwMap;
    private Claw claw;
    public ClawAct(HardwareMap hardwareMap){
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
