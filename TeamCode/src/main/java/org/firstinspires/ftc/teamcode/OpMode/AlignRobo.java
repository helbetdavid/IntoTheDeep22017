package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;

@Config
@TeleOp(name = "Align", group = "Testing")
public class AlignRobo extends LinearOpMode {

    public static double tolerance = 3.0;
    public static double kStrafe = 0;
    public static double kArmExtension = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware and subsystems
        HwMap hwMap = new HwMap();
        hwMap.init(hardwareMap);

        LimeLight limeLight = new LimeLight(hwMap.limelight, telemetry);

        // Optimize telemetry for LimeLight
        telemetry.setMsTransmissionInterval(11);

        // Wait for the start signal
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            // Get the LimeLight target offsets
            double tx = limeLight.getTargetTx();
            double ty = limeLight.getTargetTy();


            // Calculate adjustments based on tx and ty
            double x = Math.abs(tx) > tolerance ? -kStrafe * tx : 0; // Strafe adjustment
            double y = Math.abs(ty) > tolerance ? -kArmExtension * ty : 0; // Arm extension adjustment

            // Calculate motor powers for strafing
            double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
            double frontLeftPower = (y + x) / denominator;
            double backLeftPower = (y - x) / denominator;
            double frontRightPower = (y - x) / denominator;
            double backRightPower = (y + x) / denominator;

            // Set motor powers
            hwMap.leftFront.setPower(frontLeftPower);
            hwMap.leftBack.setPower(backLeftPower);
            hwMap.rightFront.setPower(frontRightPower);
            hwMap.rightBack.setPower(backRightPower);

            // Telemetry for debugging
            telemetry.addData("Aligning", "tx: %.2f, ty: %.2f", tx, ty);
            telemetry.addData("Motor Powers", "FL: %.2f, BL: %.2f, FR: %.2f, BR: %.2f",
                    frontLeftPower, backLeftPower, frontRightPower, backRightPower);
            telemetry.addData("Tolerance", tolerance);
            telemetry.addData("kStrafe", kStrafe);
            telemetry.addData("kArmExtension", kArmExtension);
            telemetry.update();

            // Prevent robot from running indefinitely if stop is requested
            if (isStopRequested()) break;
        }
    }
}
