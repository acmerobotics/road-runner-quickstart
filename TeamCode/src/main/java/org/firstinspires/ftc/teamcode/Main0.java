package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import  org.firstinspires.ftc.teamcode.Drawing;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@TeleOp(name = "#Main0")

public class Main0 extends LinearOpMode {
    private DcMotorEx lift, leftRotate, rightRotate;
    private Servo rotate, left, right;
    @Override
    public void runOpMode() throws InterruptedException {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftRotate = hardwareMap.get(DcMotorEx.class, "leftRotate");
        leftRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        rightRotate = hardwareMap.get(DcMotorEx.class, "rightRotate");
        rightRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rotate = hardwareMap.get(Servo.class, "rotate");
        left = hardwareMap.get(Servo.class, "left");
        right = hardwareMap.get(Servo.class, "right");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        left.setPosition(0.0);
        right.setPosition(0.36);
        rotate.setPosition(0.5);




        int rotatePos = 0;
        boolean pressed = false;
        waitForStart();

        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();


            rightRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRotate.setPower(1);

            telemetry.addData("rotate pos", rotatePos);
            telemetry.addData("left motor", leftRotate.getCurrentPosition());
            telemetry.addData("right motor", rightRotate.getCurrentPosition());


            telemetry.addData("left motor target", leftRotate.getTargetPosition());
            telemetry.addData("right motor target", rightRotate.getCurrentPosition());


            telemetry.addData("lift", lift.getCurrentPosition());
            telemetry.addData("right", right.getPosition());
            telemetry.addData("left", left.getPosition());
            telemetry.addData("Rotate", rotate.getPosition());
            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }

    }
}
