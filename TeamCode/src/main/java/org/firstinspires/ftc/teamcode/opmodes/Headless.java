package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.tidev2.Claw;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Elbow;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Intake;
import org.firstinspires.ftc.teamcode.hardware.tidev2.automation.BucketOperatorFSM;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Shoulder;
import org.firstinspires.ftc.teamcode.hardware.tidev2.automation.SubOperatorFSM;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Viper;


// I AM DOCTOR IVO ROBOTNIK!
@TeleOp(name = "Decapitated Robot", group = "OpModes")
public class Headless extends OpMode {


    // Insert whatever initialization your own code does
    Shoulder shoulder = new Shoulder(this);

    Elbow elbow = new Elbow(this);
    Intake intake = new Intake(this);
    Viper viper = new Viper(this);
    Claw claw = new Claw(this);

    ElapsedTime resetTimer = new ElapsedTime();


    MecanumDrive drive;

    boolean pressed_a = false;


    // Read pose
    Pose2d poseEstimate;
    Vector2d input;
    double headlessHeading;

    BucketOperatorFSM bucketOperatorFSM;
    SubOperatorFSM subOperatorFSM;


    private void gamepadToMovement() {
        float xDir = -gamepad1.left_stick_x;
        float yDir = -gamepad1.left_stick_y;


        double transXDir = xDir * Math.cos(drive.pose.heading.toDouble() - headlessHeading) - yDir * Math.sin(drive.pose.heading.toDouble() - headlessHeading);
        double transYDir = xDir * Math.sin(drive.pose.heading.toDouble() - headlessHeading) + yDir * Math.cos(drive.pose.heading.toDouble() - headlessHeading);

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
        headlessHeading = 0;
        shoulder.init();
        elbow.init();
        intake.init();
        viper.init();
        claw.init();

        resetTimer.reset();

        pressed_a = false;

        bucketOperatorFSM = new BucketOperatorFSM(gamepad2, shoulder, viper, elbow, claw, intake);
        subOperatorFSM = new SubOperatorFSM(gamepad2, shoulder, viper, elbow, claw, intake);

    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();
        poseEstimate = drive.pose;


        if (gamepad1.right_bumper) {
            headlessHeading = drive.pose.heading.toDouble();
        }


        gamepadToMovement();
        if (!gamepad1.left_bumper) {
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            input, -gamepad1.right_stick_x
                    )
            );
        } else {
            if (!gamepad1.dpad_up && !gamepad1.dpad_down && !gamepad1.dpad_left && !gamepad1.dpad_right) {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(0, 0), 0
                        )
                );
            } else if(gamepad1.dpad_up) {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(0.5, 0), 0
                        )
                );
            } else if(gamepad1.dpad_down) {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(-0.5, 0), 0
                        )
                );
            } else if(gamepad1.dpad_left) {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(0, 0.5), 0
                        )
                );
            } else if(gamepad1.dpad_right) {
                drive.setDrivePowers(
                        new PoseVelocity2d(
                                new Vector2d(0, -0.5), 0
                        )
                );
            }

        }

        bucketOperatorFSM.listen();
        subOperatorFSM.listen();
        shoulder.listen();
        shoulder.sendTelemetry();

        elbow.listen();
        intake.listen();
        viper.listen();
        claw.listen();

        elbow.sendTelemetry();
        intake.sendTelemetry();
        claw.sendTelemetry();
        viper.sendTelemetry();

        viper.passShoulderDeg(shoulder.toDegrees(shoulder.getCurrentPosition()));

        updateTelemetry(telemetry);

    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){    }
    @Override
    public void stop(){}

}
