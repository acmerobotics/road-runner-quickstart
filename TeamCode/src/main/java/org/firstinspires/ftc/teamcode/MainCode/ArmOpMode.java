package org.firstinspires.ftc.teamcode.MainCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//@TeleOp
public class ArmOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Position of the arm when it's lifted
        int armUpPosition = 1000;

        // Position of the arm when it's down
        int armDownPosition = 0;

        // Find a motor in the hardware map named "Arm Motor"
        DcMotorEx armMotor = (DcMotorEx) hardwareMap.dcMotor.get("viper_slide_left");

        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the arm to the down position
        armMotor.setTargetPosition(armDownPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            // If the A button is pressed, raise the arm
            if (gamepad1.a) {
                armMotor.setTargetPosition(armUpPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }

            // If the B button is pressed, lower the arm
            if (gamepad1.b) {
                armMotor.setTargetPosition(armDownPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }

            // Get the current position of the armMotor
            double position = armMotor.getCurrentPosition();

            // Get the target position of the armMotor
            double desiredPosition = armMotor.getTargetPosition();

            // Show the position of the armMotor on telemetry
            telemetry.addData("Encoder Position", position);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("Desired Position", desiredPosition);

            telemetry.update();
        }
    }
}