package org.firstinspires.ftc.teamcode.Actions;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.teamcode.SubSystem.ServoCam;

public class ServoCamAct {
    private HwMap hwMap;
    private ServoCam servoCam;
    private LimeLight limelight;
    public ServoCamAct(){
        hwMap = new HwMap();
        hwMap.init(hardwareMap);

        servoCam = new ServoCam(hwMap.servoCam, limelight);
    }

    public class Straigt implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            servoCam.setAngle(0.5);
            return false;
        }
    }
    public class Lateral implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket){
            servoCam.setAngle(0.0);
            return false;
        }
    }
    public Action straight(){
        return new Straigt();
    }
    public Action lateral(){
        return new Lateral();
    }
}
