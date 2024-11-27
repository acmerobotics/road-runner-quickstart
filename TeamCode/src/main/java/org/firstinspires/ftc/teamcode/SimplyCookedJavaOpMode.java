package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class SimplyCookedJavaOpMode extends LinearOpMode {
    //Declare motors
    public DcMotor FrontRight;
    public DcMotor FrontLeft;
    public DcMotor BackLeft;
    public DcMotor BackRight;
    @Override
    public void runOpMode() {
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontRight = hardwareMap.get(DcMotor.class, "BackLeft");
        telemetry.addData("Status", "Running");
        telemetry.update();
//        Wait for game to start (Driver presses PLAY)
        waitForStart();
//        Wait for game to start (Driver presses STOP)
        while (opModeIsActive());
        double PowerM  = 0;
        PowerM = -this.gamepad1.left_stick_y;
        FrontRight.setPower(PowerM);
        FrontLeft.setPower(PowerM);
        BackLeft.setPower(PowerM);
        BackRight.setPower(PowerM);
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}