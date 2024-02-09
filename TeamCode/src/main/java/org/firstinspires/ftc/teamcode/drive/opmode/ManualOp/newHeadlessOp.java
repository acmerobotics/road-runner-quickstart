package org.firstinspires.ftc.teamcode.drive.opmode.ManualOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*  this is the new headless it does not really on our hewlpers because thinking ahead each years the helper class will have to be changed.
* */

@TeleOp (name = "NEW HEADLESS OP")


public class newHeadlessOp extends OpMode {

    SampleMecanumDrive drive;


    @Override
    public void init(){
       drive = new SampleMecanumDrive(hardwareMap);
       drive.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       drive.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       drive.wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    @Override
    public void loop(){



        //headless potential

        final double x = -Math.pow(gamepad1.left_stick_x, 3.0);
        final double y = Math.pow(gamepad1.left_stick_y, 3.0);
        double rotation = gamepad1.left_trigger - gamepad1.right_trigger;

        final double direction = Math.atan2(x, y) + drive.getExternalHeading();
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));
        final double y_proc = -1 * speed * Math.sin(direction + Math.PI / 2.0);
        final double x_proc = speed * Math.cos(direction + Math.PI / 2.0);


        final double leftFront = y_proc + x_proc + rotation;
        final double leftRear = y_proc - x_proc - rotation;
        final double rightFront = y_proc - x_proc + rotation;
        final double rightRear = y_proc + x_proc - rotation;

        drive.setMotorPowers(leftFront,leftRear,rightFront,rightRear);

        if (gamepad1.triangle) {
            drive.launchServo.setPosition(.75);
        }
        if (gamepad1.circle) {
            drive.launchServo.setPosition(0);

        }


        //Gamepad2 controls the arm and claw
        //slide controls
        double vert = gamepad2.left_stick_y;
        double wrist = gamepad2.right_stick_y; //wrist range 8 at intake -100 at backdrop

        //   slideLeft.setPower(-vert); // at base 17 at backdrop 3142 (iffy number)
        //  slideRight.setPower(vert); // at base 2 at backdrop  -3125 (iffy number)
        //   wristMotor.setPower(wrist*.3);

        //the way these are called will be different in the classes with the helpers


        //limitors
        // left at base 17 at backdrop 3142 (iffy number)
        // right at base 2 at backdrop  -3125 (iffy number)
        if (drive.slideLeft.getCurrentPosition() > 3000 || drive.slideRight.getCurrentPosition() < -3000 )
        {
            drive.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.slideLeft.setPower(-vert*.5);
            drive.slideRight.setPower(vert*.5);


        } else {
            drive.slideLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.slideRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive.slideLeft.setPower(-vert);
            drive.slideRight.setPower(vert);
        }


        if (drive.wristMotor.getCurrentPosition()>=10|| drive.wristMotor.getCurrentPosition() <= -105){
            drive.wristMotor.setPower(wrist*.15);
            drive.wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            drive.wristMotor.setPower(wrist*.3);
            drive.wristMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }



        //this is the degree posistion for the servo

        //edge to edge of inner diameter of pixel is 31.75 mm //22 degrees
        //vertex to vertex is 36 mm // 25 degrees
        double closed = 0, Fopen = 50,Bopen = 50;
        double servoRot = 300;

        if (gamepad2.circle)
        {
            drive.gripServoB.setPosition(closed/servoRot);
            drive.gripServoF.setPosition(closed/servoRot);

            // gripServoB.setPosition(open/servoRot);
            //  gripServoF.setPosition(open/servoRot);

        }
        else
        {
            drive.gripServoB.setPosition(Bopen/servoRot);
            drive.gripServoF.setPosition(Fopen/servoRot);

            // gripServoB.setPosition(closed/servoRot);
            //  gripServoF.setPosition(closed/servoRot);
        }


        telemetry.addData("Lslide pos", drive.slideLeft.getCurrentPosition());
        telemetry.addData("Rslide pos", drive.slideRight.getCurrentPosition());
        telemetry.addData("wrist pos", drive.wristMotor.getCurrentPosition());
        telemetry.addData("Status", "Running");
        telemetry.update();


    }

}



