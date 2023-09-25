package org.firstinspires.ftc.teamcode.huskyteers;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.PoseVelocity2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TelemetryUtils {
    public static Telemetry telemetry;

    static void PoseVelocity2d(PoseVelocity2d pw) {
        telemetry.addData("drive", pw.component1().y);
        telemetry.addData("strafe", pw.component1().x);
        telemetry.addData("turn", pw.component2());
    }

    @SuppressLint("DefaultLocale")
    static void AprilTagDetection(AprilTagDetection detection) {
        if (detection.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
        }
    }

}
