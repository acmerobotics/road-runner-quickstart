package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name="ConceptGetPoseFromAT", group="testing")
public class ConceptGetPoseFromAT extends LinearOpMode {
    AutoFluffy fluffy;
    List<AprilTagDetection> detections;
    public void runOpMode() {
        fluffy = new AutoFluffy(this);
        waitForStart();

        while (opModeIsActive()) {
            detections = fluffy.findDetections();
            reportDetections(detections);
            reportPose("Localizer pose: ", fluffy.drive.pose);
            reportPose("AprilTag pose: ", fluffy.getPoseFromAprilTag());
            fluffy.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            fluffy.drive.updatePoseEstimate();
            telemetry.update();
        }


    }

    public void reportDetections(List<AprilTagDetection> detections) {
        if (detections == null || detections.isEmpty()) {
            RobotLog.i("no detections seen");
            telemetry.addLine("no detections seen");
        }
        else {
            for (AprilTagDetection d : detections) {
                String s;
                if (d.ftcPose == null) {
                    s = String.format("Tag ID %d, no pose available", d.id);
                }
                else {
                    s = String.format("Tag ID %d: (%.3f, %.3f) @%.3f", d.id, d.ftcPose.y, -d.ftcPose.x, d.ftcPose.yaw);
                }
                RobotLog.i(s);
                telemetry.addLine(s);
            }

        }
    }

    public void reportPose(String tag, Pose2d pose) {
        tag = tag + String.format("(%.3f, %.3f) @ %.3f", pose.position.x, pose.position.y, pose.heading.toDouble());
        RobotLog.i(tag);
        telemetry.addLine(tag);
    }

}
