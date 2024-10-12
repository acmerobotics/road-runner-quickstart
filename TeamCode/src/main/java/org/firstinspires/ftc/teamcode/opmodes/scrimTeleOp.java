package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Elbow;
import org.firstinspires.ftc.teamcode.hardware.Mouth;
import org.firstinspires.ftc.teamcode.hardware.Neck;
import org.firstinspires.ftc.teamcode.hardware.Viper;
import org.firstinspires.ftc.teamcode.hardware.Wrist;

// I AM DOCTOR IVO ROBOTNIK!
@TeleOp(name = "scrimTeleOp", group = "OpModes")
public class scrimTeleOp extends OpMode {

    // Insert whatever initialization your own code does
    Elbow elbow = new Elbow(this);
    Mouth mouth = new Mouth(this);
    Neck neck = new Neck(this);
    Viper viper = new Viper(this);
    Wrist wrist = new Wrist(this);
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

    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void stop(){}

}
