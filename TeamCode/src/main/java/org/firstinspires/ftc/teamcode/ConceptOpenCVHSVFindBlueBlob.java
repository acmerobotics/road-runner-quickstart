package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortalImpl;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Scalar;

@Config
@Autonomous(name="ConceptOpenCVHSVFindBlueBlob", group="testing")
public class ConceptOpenCVHSVFindBlueBlob extends LinearOpMode {

    final int CAMERA_WIDTH = 1920;
    final int CAMERA_HEIGHT = 1080;
    ConceptOpenCVHSV detector;
    VisionPortal portal;

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        detector = new ConceptOpenCVHSV();

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors((VisionProcessor) detector)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .build();
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Left HSV: ", formatHSValues(detector.leftHSValues));
            telemetry.addData("Center HSV: ", formatHSValues(detector.centerHSValues));
            telemetry.addData("Right HSV: ", formatHSValues(detector.rightHSValues));
            telemetry.addData("Left YCrCb: ", formatYCrCbValues(detector.leftCb));
            telemetry.addData("Center YCrCb: ", formatYCrCbValues(detector.centerCb));
            telemetry.addData("Right YCrCb: ", formatYCrCbValues(detector.rightCb));
            telemetry.update();
        }
    }

    public String formatHSValues(double[] values) {
        String s = String.format("H: %3.1f  S: %3.1f  V: %3.1f ", values[0], values[1], values[2]);
        return s;
    }

    public String formatYCrCbValues(double[] values) {
        String s = String.format("Y: %3.1f  Cr: %3.1f  Cb: %3.1f", values[0], values[1], values[2]);
        return s;
    }
}
