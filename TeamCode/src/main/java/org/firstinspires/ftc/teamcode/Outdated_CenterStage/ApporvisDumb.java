package org.firstinspires.ftc.teamcode.Outdated_CenterStage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="ApporvisDumb", group="Linear OpMode")
public class ApporvisDumb extends LinearOpMode {

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    @Override
    public void runOpMode() {

        rightBack= hardwareMap.get(DcMotor.class,"rightBack");

        leftBack= hardwareMap.get(DcMotor.class,"leftBack");

        rightFront= hardwareMap.get(DcMotor.class,"rightFront");

        leftFront= hardwareMap.get(DcMotor.class,"leftFront");

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){

            double movement = -gamepad1.left_stick_y;

            rightFront.setPower(movement);

            leftBack.setPower(movement);

            leftFront.setPower(movement);

            rightBack.setPower(movement);

            telemetry.addData("Left Front",leftFront.getPower());

            telemetry.update();
        }

    }

}
