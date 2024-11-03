//package org.firstinspires.ftc.teamcode.az.sample;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
///*
// * This OpMode illustrates an autonomous opmode using simple Odometry
// * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
// * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
// * The IMU gyro is used to stabilize the heading during all motions
// */
//
//@Autonomous(name="Sample Autonomous", group = "Mr. Phil")
//public class SampleAutonomous extends LinearOpMode
//{
//    // get an instance of the "Robot" class.
//    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
//
//    @Override public void runOpMode()
//    {
//        // Initialize the robot hardware & Turn on telemetry
//        robot.initialize(true);
//
//        // Wait for driver to press start
//        telemetry.addData(">", "Touch Play to run Auto");
//        telemetry.update();
//
//        waitForStart();
//        robot.resetHeading();  // Reset heading to set a baseline for Auto
//
//        // Run Auto if stop was not pressed.
//        if (opModeIsActive())
//        {
//            // Note, this example takes more than 30 seconds to execute, so turn OFF the auto timer.
//
//            // Drive a large rectangle, turning at each corner
//            robot.drive(  84, 0.60, 0.25);
//            robot.turnTo(90, 0.45, 0.5);
//            robot.drive(  72, 0.60, 0.25);
//            robot.turnTo(180, 0.45, 0.5);
//            robot.drive(  84, 0.60, 0.25);
//            robot.turnTo(270, 0.45, 0.5);
//            robot.drive(  72, 0.60, 0.25);
//            robot.turnTo(0, 0.45, 0.5);
//
//            sleep(500);
//
//            // Drive the path again without turning.
//            robot.drive(  84, 0.60, 0.15);
//            robot.strafe( 72, 0.60, 0.15);
//            robot.drive( -84, 0.60, 0.15);
//            robot.strafe(-72, 0.60, 0.15);
//        }
//    }
//}