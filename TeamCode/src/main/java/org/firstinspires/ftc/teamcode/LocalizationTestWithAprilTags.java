package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name="LocalizationTestWithAprilTags", group = "testing")
public class LocalizationTestWithAprilTags extends LinearOpMode {
    AutoFluffy fluffy = new AutoFluffy(this);
    @Override
    public void runOpMode() throws InterruptedException {
        boolean isBUp = true;
        MecanumDrive drive = fluffy.drive;
        List<AprilTagDetection> currentDetections;

        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            currentDetections = fluffy.findDetections();
            drive.updatePoseEstimate();
            telemetry.addLine(Pose2String(drive.pose));
            if (currentDetections == null) {
                telemetry.addLine("No AprilTags visible");
            }
            else {
                for (AprilTagDetection d : currentDetections) {
                    telemetry.addLine(Detection2String(d));
                }
            }
            telemetry.update();

            if (gamepad1.b) {
                if (isBUp) {
                    isBUp = false;
                    RobotLog.i(Pose2String(drive.pose));
                    if (currentDetections == null) {
                        RobotLog.i("No AprilTags visible");
                    }
                    else {
                        for (AprilTagDetection d: currentDetections) {
                            RobotLog.i(Detection2String(d));
                        }
                    }
                    telemetry.addLine("logged");
                }
                else {
                    isBUp = true;
                }
            }
        }

    }

    public String Pose2String(Pose2d pose) {
        String s;
        if (pose == null) {
            s="no pose available";
        }
        else {
            s=String.format("(x,y) heading: %0.3f %03.f %0.3f", pose.position.x, pose.position.y, pose.heading);
        }
        return s;
    }

    public String Detection2String(AprilTagDetection detection) {
        String s = null;
        s = "ID: " + detection.id;
        if (detection.ftcPose == null) {
            s += "    No pose available";
        }
        else {
            s += String.format("    Pose (x,y) yaw: %0.3f, %0.3f  %0.3f", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.yaw);
        }
        return s;
    }


}
