package org.firstinspires.ftc.teamcode.drive.opmode.manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SlideControl")
public class slideTest extends OpMode {
    DcMotor leftSlide, rightSlide, leftSlidePivot, rightSlidePivot; // Slide Motors

    double leftSlideStartPos;
    double rightSlideStartPos;

    double leftSlidePivStartPos;
    double rightSlidePivStartPos;

    public void init() {
        leftSlide = hardwareMap.dcMotor.get("leftSlide");
        leftSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlide = hardwareMap.dcMotor.get("rightSlide");
        rightSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlidePivot = hardwareMap.dcMotor.get("leftSlidePivot");
        leftSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightSlidePivot = hardwareMap.dcMotor.get("rightSlidePivot");
        rightSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlidePivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlidePivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftSlidePivot.setDirection(DcMotorSimple.Direction.REVERSE);

        // Starting positions of slide and pivot motors
        leftSlideStartPos = leftSlide.getCurrentPosition();
        rightSlideStartPos = rightSlide.getCurrentPosition();

        leftSlidePivStartPos = leftSlidePivot.getCurrentPosition();
        rightSlidePivStartPos = rightSlidePivot.getCurrentPosition();

    }
    public void loop() {
        telemetry.addData("Left Slide Position", leftSlide.getCurrentPosition());
        telemetry.addData("Right Slide Position", rightSlide.getCurrentPosition());

        telemetry.addData("left Pivot Position", leftSlidePivot.getCurrentPosition());
        telemetry.addData("Right Pivot Position", rightSlidePivot.getCurrentPosition());

        double pivot = gamepad2.right_stick_y;

        if (leftSlidePivot.getCurrentPosition() > (leftSlidePivStartPos + 1000) || rightSlidePivot.getCurrentPosition() <= (rightSlidePivStartPos - 2000)) {
            leftSlidePivot.setPower(pivot);
            rightSlidePivot.setPower(-pivot*.5);
        }
        else {
            leftSlidePivot.setPower(pivot);
            rightSlidePivot.setPower(-pivot *.5);
        }
    }
}
