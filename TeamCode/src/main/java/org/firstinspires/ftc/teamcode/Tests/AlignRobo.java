package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Actions.AlignAct;
import org.firstinspires.ftc.teamcode.HwMap;
import org.firstinspires.ftc.teamcode.SubSystem.LimeLight;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Aligning OpMode", group = "Test")
@Config
public class AlignRobo extends LinearOpMode {

    private HwMap hwMap;
    private LimeLight limeLight;
    private AlignAct alignAct;

    // Tuning Parameters - can be adjusted via the FTC Dashboard

    public static double tolerance = 2.0;          // Tolerance for tx and ty
    public static double kStrafe = 0;            // Strafe coefficient for adjustment
    public static double kArmExtension = 0;      // Arm extension coefficient for adjustment

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        hwMap = new HwMap();
        hwMap.init(hardwareMap);
        limeLight = new LimeLight(hwMap.limelight, telemetry);

        // Create the AlignAct action with initial tuning values
        alignAct = new AlignAct(hwMap, telemetry, 0, 0, tolerance, kStrafe, kArmExtension);

        waitForStart();

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            // Get current tx and ty values from the Limelight camera
            double tx = limeLight.getTargetTx();
            double ty = limeLight.getTargetTy();

            // Update the action with the current tx and ty
            alignAct = new AlignAct(hwMap, telemetry, tx, ty, tolerance, kStrafe, kArmExtension);

            // Run the alignment action (non-blocking)
            boolean isAligned = alignAct.run(packet);

            // Send telemetry data to the FTC Dashboard
            telemetry.addData("Aligned?", isAligned ? "Yes" : "No");
            telemetry.addData("TX", tx);
            telemetry.addData("TY", ty);
            telemetry.addData("Tolerance", tolerance);
            telemetry.addData("kStrafe", kStrafe);
            telemetry.addData("kArmExtension", kArmExtension);
            telemetry.update();

            // Send telemetry to FTC Dashboard
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        }
    }
}
