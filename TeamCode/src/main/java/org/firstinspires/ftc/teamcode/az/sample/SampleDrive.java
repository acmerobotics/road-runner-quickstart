//package org.firstinspires.ftc.teamcode.az.sample;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.az.sample.MecanumDrive;
//
//@TeleOp
//public class SampleDrive extends LinearOpMode {
//
//    public static final double DEFAULT_DRIVE_FACTOR = 1.25;
//    MecanumDrive mecanumDrive;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//
//        waitForStart();
//        telemetry.addData("Status", "Running");
//        telemetry.update();
//        while (opModeIsActive()){
//
//            double driveFactor = DEFAULT_DRIVE_FACTOR;
//
//            Vector2d vector2d = new Vector2d(
//                    -gamepad1.left_stick_x / driveFactor,
//                    -gamepad1.left_stick_x / driveFactor
//            );
//            mecanumDrive.setDrivePowers( new PoseVelocity2d(vector2d,-gamepad1.right_stick_x) );
//                    /**
//                    -gamepad1.left_stick_y / driveFactor,
//                    -gamepad1.left_stick_x / driveFactor,
//                    -gamepad1.right_stick_x**/
//
//
//            mecanumDrive.updatePoseEstimate();
//
//        }
//    }
//}
