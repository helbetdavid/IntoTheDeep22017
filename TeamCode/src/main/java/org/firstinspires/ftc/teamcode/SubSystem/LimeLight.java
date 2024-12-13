package org.firstinspires.ftc.teamcode.SubSystem;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimeLight extends SubsystemBase {
    private  Limelight3A limelight = null;
    private final Telemetry telemetry;
    private LLResult lastResult;
    private int pipeline = 0;

    public LimeLight(Limelight3A limelight, Telemetry telemetry) {
        this.limelight = limelight;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public void setPipeline(int pipeline) {
        this.pipeline = pipeline;
        limelight.pipelineSwitch(pipeline);
    }

    public LLResult getLastResult() {
        return limelight.getLatestResult();
    }

    public double getTargetTx() {
        if(limelight.getLatestResult() != null) {
            return limelight.getLatestResult().getTx();
        }
        return 0;
    }

    public double getTargetTy() {
        if(limelight.getLatestResult() != null) {
            return limelight.getLatestResult().getTy();
        }
        return 0;
    }

    public  double getAngle(){
        LLResult result = limelight.getLatestResult();
        if(result != null) {
            double[] pythonOutputs = result.getPythonOutput();
            return pythonOutputs[5];
        }
        return 0;
    }

    public void logPipelineData() {
        LLResult result = limelight.getLatestResult();
        if (result == null)
            telemetry.addLine("bag pl in capu lu david");
        else {
            double angle = getAngle();
            double tx = getTargetTx();
            double ty = getTargetTy();

            telemetry.addData("angle", angle);
            telemetry.addData("tx", tx);
            telemetry.addData("ty", ty);
            telemetry.addData("Pipeline", pipeline);

            telemetry.update();
        }
    }

}
