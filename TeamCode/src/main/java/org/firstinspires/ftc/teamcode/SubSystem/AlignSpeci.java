package org.firstinspires.ftc.teamcode.SubSystem;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class AlignSpeci extends SubsystemBase {

    private final DcMotor extendo;
    private final PIDController controllercam;

    public static double kPcam = 0.02, kIcam = 0.00025, kDcam = 0.0007;
    public static double targetcam = 34;

    private final Telemetry telemetry;

    public AlignSpeci(DcMotor extendo,Telemetry telemetry) {
        this.extendo = extendo;
        this.telemetry = telemetry;

        this.controllercam = new PIDController(kPcam, kIcam, kDcam);

    }

    public void updatePID() {
        controllercam.setP(kPcam);
        controllercam.setI(kIcam);
        controllercam.setD(kDcam);
    }

    public void alignToTarget(double targetArea) {

        double error = targetArea - targetcam;
        if (Math.abs(error) < 0.5) { // Deadband threshold
            extendo.setPower(0);
        } else {
            double power = controllercam.calculate(targetArea, targetcam);
            extendo.setPower(power);
        }

    }
}