package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.tidev2.Claw;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Elbow;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Intake;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Shoulder;
import org.firstinspires.ftc.teamcode.hardware.tidev2.ShoulderV0;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Viper;


// I AM DOCTOR IVO ROBOTNIK!
@TeleOp(name = "Decapitated Robot", group = "OpModes")
public class Headless extends OpMode {

    boolean stance;
    // Insert whatever initialization your own code does
    Shoulder shoulder = new Shoulder(this);
    ShoulderV0 shoulderV0 = new ShoulderV0(this);
    Elbow elbow = new Elbow(this);
    Intake intake = new Intake(this);
    Viper viper = new Viper(this);
    Claw claw = new Claw(this);
    // Assuming you're using StandardTrackingWheelLocalizer.java
    // Switch this class to something else (Like TwoWheeTrackingLocalizer.java) if your configuration is different
    MecanumDrive drive;
    // Set your initial pose to x: 10, y: 10, facing 90 degrees
    // Make sure to call drive.update() on *every* loop
    // Increasing loop time by utilizing bulk reads and minimizing writes will increase your odometry accuracy

//        // Retrieve your pose
//        Pose2d cur = drive.pose;
//
//        Telemetry.;
//        telemetry.addData("y", myPose.getY());
//        telemetry.addData("heading", myPose.getHeading());

    // Insert whatever teleop code you're using

    // Read pose
    Pose2d poseEstimate;
    Vector2d input;

    // Create a vector from the gamepad x/y inputs
// Then, rotate that vector by the inverse of that heading
    private void gamepadToMovement() {
        float xDir = -gamepad1.left_stick_x;
        float yDir = -gamepad1.left_stick_y;


        double transXDir = xDir * Math.cos(drive.pose.heading.toDouble()) - yDir * Math.sin(drive.pose.heading.toDouble());
        double transYDir = xDir * Math.sin(drive.pose.heading.toDouble()) + yDir * Math.cos(drive.pose.heading.toDouble());

        input = new Vector2d(
                transYDir,
                transXDir
        );
    }

// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately



    @Override
    public void init() {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        shoulder.init();
        shoulderV0.init();
        elbow.init();
        intake.init();
        viper.init();
        claw.init();

    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();
        poseEstimate = drive.pose;




        gamepadToMovement();
        if (!gamepad1.left_bumper) {
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            input, -gamepad1.right_stick_x
                    )
            );
        } else {
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(0,0), 0
                    )
            );
        }

        if (gamepad2.right_stick_y < 0) {
            shoulder.listen();
        } else {
            shoulderV0.listen();
        }
        elbow.listen();
        intake.listen();
        viper.listen();
        claw.listen();


        shoulderV0.sendTelemetry();
        intake.sendTelemetry();
        claw.sendTelemetry();


        updateTelemetry(telemetry);

    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){    }
    @Override
    public void stop(){}

}
