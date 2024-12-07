package org.firstinspires.ftc.teamcode.SubSystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends SubsystemBase {
    private final PIDFController controller;
    private DcMotor rightLift;
    private DcMotor leftLift;
    private final Telemetry telemetry;
    public static double kP = 0.01, kI=0, kD=0.0001, kF=0;
    public static double target=0;

    public Lift(DcMotor rightLift, DcMotor leftLift, Telemetry telemetry) {
        this.rightLift = rightLift;
        this.leftLift = leftLift;
        this.telemetry = telemetry;

        this.controller = new PIDFController(kP,kI,kD,kF);

        this.rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updatePIDGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

        controller.setPIDF(kP,kI,kD,kF);
    }

    public void setTarget(double target) {
        this.target = target;
    }

    public void setPower() {
        double pos = rightLift.getCurrentPosition();
        double calcul = controller.calculate(pos,target);

        rightLift.setPower(calcul);
        leftLift.setPower(calcul);

        telemetry.addData("target",target);
        telemetry.addData("pos",pos);
        telemetry.update();
    }

    public double getPosition(){
        return rightLift.getCurrentPosition();
    }
}
