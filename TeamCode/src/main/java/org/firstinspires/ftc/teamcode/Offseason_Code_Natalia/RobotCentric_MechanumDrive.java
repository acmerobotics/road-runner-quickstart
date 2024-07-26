package org.firstinspires.ftc.teamcode.Offseason_Code_Natalia;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="RobotCentric_MechanumDrive", group="Linear OpMode")
public class RobotCentric_MechanumDrive extends LinearOpMode {
    private DcMotor LeftFront;
    private DcMotor RightFront;
    private DcMotor LeftBack;
    private DcMotor RightBack;
    //claw and shoulder stuff
    private Servo claw;
    private Servo claw2;

    // this code is robot-centric. IDK how it is to drive but.... gl
    @Override
    public void runOpMode() {
        LeftBack= hardwareMap.get(DcMotor.class,"LeftBack");
        RightBack= hardwareMap.get(DcMotor.class,"RightBack");
        LeftFront= hardwareMap.get(DcMotor.class,"LeftBack");
        RightFront= hardwareMap.get(DcMotor.class,"RightFront");

        LeftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        RightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        RightFront.setDirection(DcMotorSimple.Direction.REVERSE);


        //claw and shoulder
        hardwareMap.get(DcMotor.class, "shoulder");

        waitForStart();
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // counteract imperfect strafing apparently?
            double rx = gamepad1.right_stick_x;

            LeftFront.setPower(y+x+rx);
            LeftBack.setPower(y-x+rx);
            RightFront.setPower(y-x-rx);
            RightBack.setPower(y+x-rx);

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double LeftFrontPower =(y + x + rx)/ denominator;
            double LeftBackPower = (y- x + rx)/ denominator;
            double RightFrontPower= (y-x-rx)/ denominator;
            double RightBackPower = (y+x-rx)/ denominator;

            RightFront.setPower(RightFrontPower);
            LeftFront.setPower(LeftFrontPower);
            RightBack.setPower(RightBackPower);
            LeftBack.setPower(LeftBackPower);

        }
        telemetry.addData("Left Front", LeftFront.getPower());
        telemetry.addData("RightFront", RightFront.getPower());

        telemetry.update();
    }

}

//this code was made by natalia (not natalie)//