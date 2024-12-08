package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends SubsystemBase {
    private PIDFController controller;
    public double kP = 0.006, kI = 0.0003, kD = 0, kF = 0.0001;
    public double target = 0;
    private DcMotor motorStanga, motorDreapta;
    private final Telemetry telemetry;

    public Lift(DcMotor stanga, DcMotor dreapta, Telemetry telemetry) {
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.motorDreapta = dreapta;
        this.motorStanga = stanga;
        this.controller = new PIDFController(kP, kI, kD, kF);

        this.motorStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorDreapta.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motorStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.motorDreapta.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updatePIDFGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setPower() {
        double position = motorDreapta.getCurrentPosition();
        double output = controller.calculate(position, target);

        motorStanga.setPower(output);
        motorDreapta.setPower(output);

        telemetry.addData("target", target);
        telemetry.addData("pos", position);
        telemetry.update();
    }

    public double getPosition(){
        return motorDreapta.getCurrentPosition();
    }
}
