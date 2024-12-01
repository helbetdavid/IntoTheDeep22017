package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HwMap {
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightBack = null;
    public DcMotor rightFront = null;
    public DcMotor extendo = null;
    private HardwareMap hwMap = null;
    public Limelight3A limelight;
    public Servo servoCam;


    public void init(HardwareMap hwMap) {
        // Save reference to hardware map
        this.hwMap = hwMap;

        // Drivetrain motors
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        leftBack = hwMap.get(DcMotorEx.class, "leftBack");
        rightBack = hwMap.get(DcMotorEx.class, "rightBack");
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);


        limelight = hwMap.get(Limelight3A.class, "limelight");

        // Extendo
        extendo = hwMap.get(DcMotor.class, "extendo");
        extendo.setDirection(DcMotorSimple.Direction.REVERSE);

        // ServoCam
        servoCam = hwMap.get(Servo.class,"servoCam");

    }
    public HardwareMap getHwMap() {
        return hwMap;
    }
}
