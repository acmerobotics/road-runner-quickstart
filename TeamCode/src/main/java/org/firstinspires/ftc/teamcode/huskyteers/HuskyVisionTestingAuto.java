package org.firstinspires.ftc.teamcode.huskyteers;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Optional;

@Autonomous(name = "Husky Vision Testing Autonomous", group = "Autonomous")
public class HuskyVisionTestingAuto extends LinearOpMode {
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
            PoseVelocity2d pw = huskyBot.alignWithAprilTag(1);
            huskyBot.driveRobot(pw.component1().y, pw.component1().x, -pw.component2(), 1.0);

            telemetry.addData("Drive", pw.component1().y);
            telemetry.addData("Strafe", pw.component1().x);
            telemetry.addData("Turn", pw.component2());

            Optional<AprilTagDetection> closestAprilTag = huskyBot.huskyVision.backdropAprilTagDetection.closestAprilTag();
            Optional<AprilTagDetection> aprilTag1 = huskyBot.huskyVision.backdropAprilTagDetection.getAprilTagById(1);

            // If closestAprilTag/aprilTag1 exists, add the telemetry for that. If not, do nothing
            closestAprilTag.ifPresent(aprilTagDetection -> telemetry.addData("Closest April Tag Range", aprilTagDetection.ftcPose.range));
            aprilTag1.ifPresent(aprilTagDetection -> telemetry.addData("April Tag ID 1:", aprilTagDetection.ftcPose.range));

            telemetry.update();

            // share the cpu
            sleep(20);
        }
        // endregion
    }
}
