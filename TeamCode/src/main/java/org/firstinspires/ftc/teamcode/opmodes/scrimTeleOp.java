//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//
//public class scrimTeleOp extends OpMode {
//
//    // Insert whatever initialization your own code does
//
//    // Assuming you're using StandardTrackingWheelLocalizer.java
//    // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
//    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//    // Set your initial pose to x: 10, y: 10, facing 90 degrees
//        // Make sure to call drive.update() on *every* loop
//        // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy
//
//        // Retrieve your pose
//        Pose2d myPose = drive.pose;
//
//        telemetry.addData("x", myPose.getX());
//        telemetry.addData("y", myPose.getY());
//        telemetry.addData("heading", myPose.getHeading());
//
//        // Insert whatever teleop code you're using
//
//    // Read pose
//    Pose2d poseEstimate = drive.pose;
//
//    // Create a vector from the gamepad x/y inputs
//// Then, rotate that vector by the inverse of that heading
//    Vector2d input = new Vector2d(
//            -gamepad1.left_stick_y,
//            -gamepad1.left_stick_x
//    ).(-poseEstimate.getHeading());
//
//// Pass in the rotated input + right stick value for rotation
//// Rotation is not part of the rotated input thus must be passed in separately
//drive.setWeightedDrivePower(
//        new Pose2d(
//            input.getX(),
//                input.getY(),
//                        -gamepad1.right_stick_x
//        )
//                );
//
//
//    @Override
//    public void init() {
//
//    }
//
//    @Override
//    public void loop() {
//
//    }
//}
