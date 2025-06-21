package org.firstinspires.ftc.teamcode.drive.opmode.manual;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "PivotControl")
public class pivotTest extends OpMode {
    DcMotor leftSlidePivot;
    DcMotor rightSlidePivot;
    public void init() {
        leftSlidePivot = hardwareMap.dcMotor.get("leftSlidePivot");
        rightSlidePivot = hardwareMap.dcMotor.get("rightSlidePivot");
        leftSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightSlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void loop() {
        double leftPiv = gamepad2.left_stick_y;
        leftSlidePivot.setPower(-leftPiv);

        double rightPiv = gamepad2.right_stick_y;
        rightSlidePivot.setPower(-rightPiv);
    }
}
