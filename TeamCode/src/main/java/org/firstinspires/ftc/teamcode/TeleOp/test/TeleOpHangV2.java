package org.firstinspires.ftc.teamcode.TeleOp.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.LeftActuator;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.LeftActuatorServo;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.RightActuator;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.RightActuatorServo;
import org.firstinspires.ftc.teamcode.mechanisms.v1.Arm;


@TeleOp(name="Hang Test V2", group="test")
@Config
public class TeleOpHangV2 extends LinearOpMode {

    int leftActuatorPosition = LeftActuator.ACTUATOR_COLLAPSED;
    int rightActuatorPosition = RightActuator.ACTUATOR_COLLAPSED;

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Arm arm = new Arm(hardwareMap);

        RightActuatorServo rightArmServo = new RightActuatorServo(hardwareMap);
        LeftActuatorServo leftArmServo = new LeftActuatorServo(hardwareMap);

        LeftActuator leftActuator = new LeftActuator(hardwareMap);
        RightActuator rightActuator = new RightActuator(hardwareMap);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.x){
                rightArmServo.setHorizontal();
            }
            else if (gamepad1.y){
                rightArmServo.setVertical();
            }

            if (gamepad1.a){
                leftArmServo.setHorizontal();
            }
            else if (gamepad1.b){
                leftArmServo.setVertical();
            }

            if (gamepad1.dpad_right){
                leftArmServo.setVertical();
                rightArmServo.setVertical();
            } else if (gamepad1.dpad_left){
                leftArmServo.setHanging();
                rightArmServo.setHanging();
            }

            if (gamepad1.dpad_up){
                leftActuatorPosition = LeftActuator.ACTUATOR_UP;
                rightActuatorPosition = RightActuator.ACTUATOR_UP;
            } else if (gamepad1.dpad_down){
                leftActuatorPosition = LeftActuator.ACTUATOR_COLLAPSED;
                rightActuatorPosition = RightActuator.ACTUATOR_COLLAPSED;

            }

            leftActuator.motor.setTargetPosition(leftActuatorPosition);
            leftActuator.motor.setVelocity(1000);
            leftActuator.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            rightActuator.motor.setTargetPosition(rightActuatorPosition);
            rightActuator.motor.setVelocity(1000);
            rightActuator.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            /* send telemetry to the driver of the arm's current position and target position */
            telemetry.addData("left Actuator Target", leftActuator.motor.getTargetPosition());
            telemetry.addData("left Actuator Current", leftActuator.motor.getCurrentPosition());
            telemetry.addData("right Actuator Target", rightActuator.motor.getTargetPosition());
            telemetry.addData("right Actuator Current", rightActuator.motor.getCurrentPosition());
            telemetry.addData("left Servo", leftArmServo.servo.getPosition());
            telemetry.addData("right Servo", leftArmServo.servo.getPosition());

            //telemetry.addData("lift position", lift.motor.getCurrentPosition());
            telemetry.update();

        }

    }

    private double squaredInputWithSign(double input) {
        double output = input * input;
        if (input<0){
            output = -output;
        }
        return output;
    }
}
