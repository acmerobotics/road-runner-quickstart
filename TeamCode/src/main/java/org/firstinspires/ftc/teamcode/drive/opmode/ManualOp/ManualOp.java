package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

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

    Servo launchServo;




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
        launchServo = hardwareMap.servo.get("launchServo");
        gripServoB.setPosition(0);
        gripServoF.setPosition(0);

        // leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        //  rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData("Servo Position", gripServoB.getPosition());
    }

        public void loop() {
        //Game pad 1
            /**
             * the left joystick moving left or right is strafe
             * moving
             */
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.left_trigger - gamepad1.right_trigger;

        leftFront.setPower((-y - x + rx));
        leftRear.setPower((-y + x + rx));
        rightFront.setPower((y - x + rx));
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

            //edge to edge of inner diameter of pixel is 31.75 mm //22 degrees
            //vertex to vertex is 36 mm // 25 degrees
            double closed = 0, open = 40;
            double servoRot = 300;

            if (gamepad2.circle)
            {
                gripServoB.setPosition(open/servoRot);
                gripServoF.setPosition(open/servoRot);

            }
            else
            {
                gripServoB.setPosition(closed/servoRot);
                gripServoF.setPosition(closed/servoRot);
            }






            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Status", "Running");
            telemetry.update();

    }
}
