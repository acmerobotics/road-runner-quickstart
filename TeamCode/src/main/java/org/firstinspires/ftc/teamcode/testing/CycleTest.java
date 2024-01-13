package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.*;
import java.util.function.Supplier;

//@TeleOp(name="CycleTest", group="Linear Opmode")

public class CycleTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor intake_elbow = hardwareMap.get(DcMotor.class, "intake_elbow");
        intake_elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        DcMotor intake_grabber = hardwareMap.get(DcMotor.class, "intake_grabber");
        intake_grabber.setDirection(DcMotorSimple.Direction.FORWARD);
        DcMotor outtake_elbow = hardwareMap.get(DcMotor.class, "outtake_elbow");
        outtake_elbow.setDirection(DcMotorSimple.Direction.FORWARD);
        DcMotor hang_arm = hardwareMap.get(DcMotor.class, "hang_arm");
        hang_arm.setDirection(DcMotorSimple.Direction.FORWARD);
        Servo left_intake = hardwareMap.get(Servo.class, "left_intake");
        left_intake.setDirection(Servo.Direction.FORWARD);
        Servo right_intake = hardwareMap.get(Servo.class, "right_intake");
        right_intake.setDirection(Servo.Direction.FORWARD);
        Servo outtake_wrist = hardwareMap.get(Servo.class, "outtake_wrist");
        outtake_wrist.setDirection(Servo.Direction.FORWARD);
        Servo drone_launcher = hardwareMap.get(Servo.class, "drone_launcher");
        drone_launcher.setDirection(Servo.Direction.FORWARD);


        int i = 0;
        int p = 0;
        List<DcMotor> activeMotor = new ArrayList<DcMotor>();
        List<Servo> activeServo = new ArrayList<Servo>();
        activeMotor.add(intake_elbow);
        activeMotor.add(intake_grabber);
        activeMotor.add(outtake_elbow);
        activeMotor.add(hang_arm);
        activeServo.add(left_intake);
        activeServo.add(right_intake);
        activeServo.add(outtake_wrist);
        activeServo.add(drone_launcher);
        waitForStart();
        while (opModeIsActive()) {
            activeMotor.get(i).setPower(gamepad1.left_stick_y);
            activeServo.get(p).setPosition(gamepad1.right_stick_y);
            telemetry.addData("Motor", activeMotor.get(i));
            telemetry.addData("Servo", activeServo.get(p));
            if (gamepad1.right_bumper) {
                if (i == 3) {
                    i = -1;
                }
                i += 1;
            }
            if (gamepad1.left_bumper) {
                if (p == 3) {
                    p = -1;
                }
                p += 1;
            }
        }
    }
}