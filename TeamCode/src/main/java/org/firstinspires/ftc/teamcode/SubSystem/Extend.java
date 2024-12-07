package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Extend extends SubsystemBase {
    private final PIDController controller;
    private DcMotor extendo;
    private final Telemetry telemetry;
    public double kP = 0.008, kI = 0.00001, kD = 0.0001;
    public double target = 0;


    public Extend(DcMotor extendo, Telemetry telemetry) {
        this.extendo = extendo;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.controller = new PIDController(kP, kI, kD);

        this.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void updatePIDGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
    }
    public void setTarget(double target) {
        this.target = target;
    }

    public void setPower() {
        double currentPosition = extendo.getCurrentPosition();
        double output = controller.calculate(currentPosition, target);
        extendo.setPower(output);

        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Output", output);
        telemetry.update();
    }
    public double getPosition(){
        return extendo.getCurrentPosition();
    }
}
