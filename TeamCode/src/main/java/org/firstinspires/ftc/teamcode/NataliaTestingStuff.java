package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="NataliaTestingStuff", group="Linear OpMode")
public class NataliaTestingStuff extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor LeftBack;
    private DcMotor RightBack;
    //claw and shoulder stuff
    private Servo claw;
    private Servo claw2;
    private Servo wrist;
    private DcMotor shoulder;

    double shoulderStick;
    int shoulder_Position;


    @Override
    public void runOpMode() {
        LeftBack= hardwareMap.get(DcMotor.class,"LeftBack");
        RightBack= hardwareMap.get(DcMotor.class,"RightBack");
        LeftBack= hardwareMap.get(DcMotor.class,"LeftBack");
        LeftBack= hardwareMap.get(DcMotor.class,"LeftBack");

        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        shoulder.setDirection(DcMotorSimple.Direction.FORWARD);

        //claw and shoulder
        hardwareMap.get(DcMotor.class, "shoulder");

        waitForStart();
        while (opModeIsActive()) {

         double movement= -gamepad1.left_stick_y;

            RightFront.setPower(movement);

            RightBack.setPower(movement);

            LeftBack.setPower(movement);

            LeftFront.setPower(movement);
            telemetry.addData("Left Front", LeftFront.getPower());

         if(gamepad1.right_stick_y>0) {
             shoulder_Position += gamepad1.right_stick_y;
             shoulder.setTargetPosition(shoulder_Position);
             shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             shoulder.setPower(1);
         }else if (gamepad1.right_stick_y < 0) {
             shoulder_Position += gamepad1.right_stick_y;
             shoulder.setTargetPosition(shoulder_Position);
             shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
             shoulder.setPower(.5);
             }
            telemetry.addData("Left Front", LeftFront.getPower());

            telemetry.update();
         }

        }

    }
//this code was made by natalia (not natalie)//