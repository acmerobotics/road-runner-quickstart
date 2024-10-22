package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.tidev1_DO_NOT_USE.Elbow;
import org.firstinspires.ftc.teamcode.hardware.tidev1_DO_NOT_USE.Mouth;
import org.firstinspires.ftc.teamcode.hardware.tidev1_DO_NOT_USE.Neck;
import org.firstinspires.ftc.teamcode.hardware.tidev1_DO_NOT_USE.Viper;
import org.firstinspires.ftc.teamcode.hardware.tidev1_DO_NOT_USE.Wrist;

// I AM DOCTOR IVO ROBOTNIK!
@TeleOp(name = "scrimTeleOp", group = "OpModes")
public class scrimTeleOp extends OpMode {

    // Insert whatever initialization your own code does
    Elbow elbow;
    Mouth mouth;
    Neck neck;
    Viper viper;
    Wrist wrist;
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

        //translated vector for headlessness. do later.
        float transXDir;
        float transYDir;

        input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        );
    }

// Pass in the rotated input + right stick value for rotation
// Rotation is not part of the rotated input thus must be passed in separately



    @Override
    public void init() {

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        elbow = new Elbow(this);
        mouth = new Mouth(this);
        neck = new Neck(this);
        viper = new Viper(this);
        wrist = new Wrist(this);

        elbow.init();
        mouth.init();
        neck.init();
        viper.init();
        wrist.init();

    }

    @Override
    public void loop() {
        poseEstimate = drive.pose;





        gamepadToMovement();
        drive.setDrivePowers(
                new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x
                        ), -gamepad1.right_stick_x
                )
        );

        elbow.listen();
        mouth.listen();
        neck.listen();
        viper.listen();
        wrist.listen();

        elbow.sendTelemetry();
        mouth.sendTelemetry();
        neck.sendTelemetry();
        viper.sendTelemetry();
        wrist.sendTelemetry();
        updateTelemetry(telemetry);

    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void stop(){}

}
