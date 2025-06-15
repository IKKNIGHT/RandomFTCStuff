package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.IPLSmoothner;

import java.util.ArrayList;
import java.util.List;

@TeleOp()
public class IPL_Tester extends LinearOpMode {
    DistanceSensor distance;
    IPLSmoothner smoothner;
    List<Double> distances;

    @Override
    public void runOpMode() throws InterruptedException {
        distance = hardwareMap.get(DistanceSensor.class, "distance");
        smoothner = new IPLSmoothner(IPL_Constants.gamma, IPL_Constants.delta);
        distances = new ArrayList<>();  // initialize list
        FtcDashboard dashboard = FtcDashboard.getInstance();
        waitForStart();

        while (opModeIsActive()) {
            smoothner.setGamma(IPL_Constants.gamma);
            smoothner.setDelta(IPL_Constants.delta);
            TelemetryPacket packet = new TelemetryPacket();
            // Read distance
            double currentDistance = distance.getDistance(DistanceUnit.MM);
            distances.add(currentDistance);

            // Calculate filtered value safely with correct window size
            int actualWindowSize = Math.min(distances.size() - 1, IPL_Constants.windowSize);
            double filtered = smoothner.getFilteredValue(distances, distances.size() - 1, actualWindowSize);

            // Display telemetry
            telemetry.addData("Distance", currentDistance);
            telemetry.addData("Filtered Distance", filtered);
            packet.put("Distance", currentDistance);
            packet.put("Filtered Distance", filtered);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }

    @Config()
    public static class IPL_Constants {
        public static double gamma = 0.5;  // safer default
        public static double delta = 1.5;
        public static int windowSize = 10;
    }
}
