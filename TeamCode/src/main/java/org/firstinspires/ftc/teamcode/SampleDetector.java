package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.utils.LimelightConstants;

// this class uses color pipelines!

public class SampleDetector extends LinearOpMode {

    Limelight3A limelight;

    double tx,ty,ta;

    int defaultPipeline = LimelightConstants.BLUE_PIPELINE;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, LimelightConstants.LIMELIGHT_CONFIG_NAME);
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!

        telemetry.addLine("Press A or B to change pipeline. Where A is Blue, and B is Red.");

        while(opModeInInit()){

            telemetry.addData("Pipeline", limelight.getLatestResult().getPipelineIndex());

            if (gamepad1.a){
                defaultPipeline = LimelightConstants.BLUE_PIPELINE;
            }
            else if (gamepad1.b){
                defaultPipeline = LimelightConstants.RED_PIPELINE;
            }

            telemetry.update();
        }
        while(opModeIsActive()){

            // first get results of defaultpipeline then yellow pipeline
            limelight.pipelineSwitch(defaultPipeline);
            LLResult result = limelight.getLatestResult();
            if(limelight.getLatestResult().getPipelineIndex() == defaultPipeline){ // only if we have already switched to the default pipeline
                if (result != null && result.isValid()) {
                    tx = result.getTx(); // How far left or right the target is (degrees)
                    ty = result.getTy(); // How far up or down the target is (degrees)
                    ta = result.getTa(); // How big the target looks (0%-100% of the image)

                    telemetry.addData("Target X", tx);
                    telemetry.addData("Target Y", ty);
                    telemetry.addData("Target Area", ta);
                } else {

                    if(defaultPipeline == LimelightConstants.BLUE_PIPELINE){
                        telemetry.addData("Limelight", "No Blue Targets");
                    }else{
                        telemetry.addData("Limelight", "No Red Targets");
                    }
                    // switch to yellow as a fallback
                    limelight.pipelineSwitch(LimelightConstants.YELLOW_PIPELINE);
                    result = limelight.getLatestResult();
                    if (result != null && result.isValid()) {
                        tx = result.getTx(); // How far left or right the target is (degrees)
                        ty = result.getTy(); // How far up or down the target is (degrees)
                        ta = result.getTa(); // How big the target looks (0%-100% of the image)

                        telemetry.addData("Target X", tx);
                        telemetry.addData("Target Y", ty);
                        telemetry.addData("Target Area", ta);
                    }else {
                        telemetry.addData("Limelight", "No Targets");
                    }
                }
            }
        }
    }
}
