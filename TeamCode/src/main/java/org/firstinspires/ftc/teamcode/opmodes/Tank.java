package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware.tidev2.Shoulder;


// I AM DOCTOR IVO ROBOTNIK!
@TeleOp(name = "\"Tank\" by Seatbelts ", group = "OpModes")
public class Tank extends OpMode {

    // Insert whatever initialization your own code does

    int max;
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    DcMotorEx rightFront;

    Shoulder shoulder = new Shoulder(this);

    MecanumDrive drive;

    Pose2d poseEstimate;
    Vector2d input;




    @Override
    public void init() {
        max = 1;

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front_drive");
    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();
        poseEstimate = drive.pose;



        float Lx = -gamepad1.left_stick_y;
        float Rx = -gamepad1.right_stick_y;
        leftFront.setPower(Lx * max);
        leftBack.setPower(Lx * max);
        rightFront.setPower(Rx * max);
        rightBack.setPower(Rx * max);


        if (gamepad1.left_trigger != 0 && gamepad1.right_trigger != 0) {
            ;
        } else if (gamepad1.left_trigger != 0) {
            leftFront.setPower(-max * gamepad1.left_trigger);
            leftBack.setPower(max * gamepad1.left_trigger);
            rightFront.setPower(max * gamepad1.left_trigger);
            rightBack.setPower(-max * gamepad1.left_trigger);
        } else if (gamepad1.right_trigger != 0) {
            leftFront.setPower(max * gamepad1.right_trigger);
            leftBack.setPower(-max * gamepad1.right_trigger);
            rightFront.setPower(-max * gamepad1.right_trigger);
            rightBack.setPower(max * gamepad1.right_trigger);
        }


        updateTelemetry(telemetry);

    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){    }
    @Override
    public void stop(){}

}
