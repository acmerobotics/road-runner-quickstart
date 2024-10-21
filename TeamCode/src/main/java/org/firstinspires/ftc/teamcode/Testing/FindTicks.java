package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Claw;
import org.firstinspires.ftc.teamcode.mechanisms.Extendo;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Slides;

@TeleOp
public class FindTicks extends LinearOpMode {
    Extendo extendo = new Extendo(hardwareMap);
    Slides slides = new Slides(hardwareMap);
    Claw claw = new Claw(hardwareMap);
    Intake intake = new Intake(hardwareMap);

    @Override
    public void runOpMode() {

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("extendo", extendo.getPos());
            telemetry.addData("slides", slides.getPos());
            telemetry.addData("clawServo", claw.getBucketPos());
            telemetry.addData("intakeLeftServo", intake.intakeServoLeft);
            telemetry.addData("intakeRightServo", intake.intakeServoRight);
            telemetry.update();
        }
    }
}
