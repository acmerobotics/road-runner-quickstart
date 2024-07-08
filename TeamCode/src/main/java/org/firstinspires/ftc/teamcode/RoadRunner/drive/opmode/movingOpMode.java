package org.firstinspires.ftc.teamcode.RoadRunner.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;


@TeleOp

public class movingOpMode extends LinearOpMode
{
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;

    @Override
    public void runOpMode() throws InterruptedException
    {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.a)
                leftFront.setPower(1);
            else if(gamepad1.b)
                rightFront.setPower(1);
            else if(gamepad1.y)
                rightRear.setPower(1);
            else if(gamepad1.x)
                leftRear.setPower(1);
            else
            {
                double rotRight = 0, rotLeft = 0;
                if(gamepad1.right_stick_x > 0) {
                    rotRight = gamepad1.right_stick_x;
                } else {
                    rotLeft = 0 - gamepad1.right_stick_x;
                }
                //double rotPower = (g.right_trigger - g.left_trigger) * ROTATION_MULTIPLIER;
                rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


                rightRear.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x - rotRight + rotLeft);
                rightFront.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x - rotRight + rotLeft);
                leftFront.setPower(-gamepad1.left_stick_y + gamepad1.left_stick_x + rotRight - rotLeft);
                leftRear.setPower(-gamepad1.left_stick_y - gamepad1.left_stick_x + rotRight - rotLeft);
            }



            drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
