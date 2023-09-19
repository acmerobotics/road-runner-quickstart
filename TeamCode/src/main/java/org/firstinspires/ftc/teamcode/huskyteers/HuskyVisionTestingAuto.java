package org.firstinspires.ftc.teamcode.huskyteers;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Optional;

@Autonomous(name = "Husky Vision Testing Autonomous", group = "Autonomous")
public class HuskyVisionTestingAuto extends LinearOpMode {
    @SuppressLint("DefaultLocale") // To get rid of the warnings when using String.format
    @Override
    public void runOpMode() {
        // region INITIALIZATION
        HuskyBot huskyBot = new HuskyBot(this);
        huskyBot.init();

        waitForStart();
        if (isStopRequested()) return;
        // endregion

        // region TELEOP LOOP
        while (opModeIsActive() && !isStopRequested()) {
            PoseVelocity2d pw = huskyBot.alignWithAprilTag(583);
            huskyBot.driveRobot(pw.component1().y, pw.component1().x, -pw.component2(), 1.0);

            telemetry.addData("Drive", pw.component1().y);
            telemetry.addData("Strafe", pw.component1().x);
            telemetry.addData("Turn", pw.component2());

            Optional<AprilTagDetection> closestAprilTag = huskyBot.huskyVision.backdropAprilTagDetection.closestAprilTag();
            List<AprilTagDetection> aprilTagDetectionList = huskyBot.huskyVision.backdropAprilTagDetection.aprilTag.getDetections();

            // If closestAprilTag/aprilTag1 exists, add the telemetry for that. If not, do nothing
            closestAprilTag.ifPresent(detection -> telemetry.addData(String.format("Closest April Tag (%s) Range", detection.id), detection.ftcPose.range));

            for (AprilTagDetection detection : aprilTagDetectionList) {
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

            telemetry.update();

            // share the cpu
            sleep(20);
        }
        // endregion
    }
}
