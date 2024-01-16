package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuel;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.ManualOp.manuelHelpers.Controller;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.WaitSegment;

//@Disabled

/** This is what workd **/


@TeleOp(name = "USE THIS ONE - Manual Op")
public class ManualOp extends OpMode {

    //Here we are creating all the parts what we will manipulate
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor slideLeft;
    DcMotor slideRight;

    DcMotor wristMotor;
    Servo gripServoF;
    Servo gripServoB;





    public void init() {
        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        slideLeft = hardwareMap.dcMotor.get("slideLeft");
        slideRight = hardwareMap.dcMotor.get("slideRight");
        wristMotor = hardwareMap.dcMotor.get("wristMotor");
        gripServoF = hardwareMap.servo.get("gripServoF");
        gripServoB = hardwareMap.servo.get("gripServoB");

        gripServoB.setPosition(0);
        gripServoF.setPosition(0);
        //reverse  the
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Servo Position", gripServoB.getPosition());
    }

        public void loop() {
        //Game pad 1
        //base controls
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.left_trigger - gamepad1.right_trigger;

        leftFront.setPower(-(y + x - rx));
        leftRear.setPower(-(y - x + rx));
        rightFront.setPower((y - x - rx));
        rightRear.setPower((y + x + rx));



        //Gamepad2 controls the arm and claw
            //slide controls
        double vert = -gamepad2.left_stick_y;
        double wrist = gamepad2.right_stick_x;

        slideLeft.setPower(-vert);
        slideRight.setPower(vert);
        wristMotor.setPower(wrist);

        //the way these are called will be different in the classes with the helpers

        slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double tgtPower = gamepad2.left_trigger;

        //this is the degree posistion for the servo
            double frontposClose = 100, backposClose = 100;
            double frontposOpen = 120 , backposOpen = 120;
            double servoRot = 300;
            int pressed = 0;

            if (gamepad2.circle)
            {
                pressed++;
            }
            else
            {
                pressed =0;
            }


            if (pressed % 2 == 0)
            {

                //.5 = 90
                gripServoB.setPosition(backposOpen/servoRot);
                gripServoF.setPosition(frontposOpen/servoRot);
            }
            else
            {
                gripServoF.setPosition(frontposClose/servoRot);
                gripServoB.setPosition(backposClose/servoRot);
            }



            telemetry.addData("Servo Position", gripServoB.getPosition());
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Status", "Running");
            telemetry.update();

    }
}
