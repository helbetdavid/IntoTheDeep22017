package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class ExtendNou extends SubsystemBase {

    private final DcMotor extendo;
    private final PIDController controller;

    public static double kP = 0.045, kI = 0, kD = 0.0007;
    public static int target = 0;

    public ExtendNou(DcMotor extendo) {
        this.extendo = extendo;
        this.controller = new PIDController(kP, kI, kD);
        this.extendo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void updatePID() {
        controller.setP(kP);
        controller.setI(kI);
        controller.setD(kD);
    }

    public void runToTarget(double target) {
        double currentPosition = extendo.getCurrentPosition();
        double power = controller.calculate(currentPosition, target);
        extendo.setPower(power);
    }

    public double getCurrentPosition() {
        return extendo.getCurrentPosition();
    }
}
