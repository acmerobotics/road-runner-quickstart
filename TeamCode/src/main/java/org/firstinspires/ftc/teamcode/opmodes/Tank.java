package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.tidev2.Claw;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Elbow;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Intake;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Shoulder;
import org.firstinspires.ftc.teamcode.hardware.tidev2.ShoulderV0;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Viper;


// I AM DOCTOR IVO ROBOTNIK!
@TeleOp(name = "\"Tank\" by Seatbelts ", group = "OpModes")
public class Tank extends OpMode {

    // Insert whatever initialization your own code does

    int max;
    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    DcMotorEx rightFront;

    Shoulder shoulder = new Shoulder(this);
    ShoulderV0 shoulderV0 = new ShoulderV0(this);
    Elbow elbow = new Elbow(this);
    Intake intake = new Intake(this);
    Viper viper = new Viper(this);
    Claw claw = new Claw(this);

    MecanumDrive drive;

    Pose2d poseEstimate;
    double speed;

    ElapsedTime speedTimer = new ElapsedTime();




    @Override
    public void init() {
        max = 1;

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front_drive");

        shoulderV0.init();
        elbow.init();
        intake.init();
        viper.init();
        claw.init();
        speed = 1;
    }

    @Override
    public void loop() {
        drive.updatePoseEstimate();
        poseEstimate = drive.pose;



        float Lx = -gamepad1.left_stick_y;
        float Rx = -gamepad1.right_stick_y;
        leftFront.setPower(Lx * max * speed);
        leftBack.setPower(Lx * max * speed);
        rightFront.setPower(Rx * max * speed);
        rightBack.setPower(Rx * max * speed);


        if (gamepad1.left_trigger != 0 && gamepad1.right_trigger != 0) {
            ;
        } else if (gamepad1.left_trigger != 0) {
            leftFront.setPower(-max * gamepad1.left_trigger);
            leftBack.setPower(max * gamepad1.left_trigger);
            rightFront.setPower(max * gamepad1.left_trigger);
            rightBack.setPower(-max * gamepad1.left_trigger);
        } else if (gamepad1.right_trigger != 0) {
            leftFront.setPower(max * gamepad1.right_trigger);
            leftBack.setPower(-max * gamepad1.right_trigger);
            rightFront.setPower(-max * gamepad1.right_trigger);
            rightBack.setPower(max * gamepad1.right_trigger);
        }

        if (gamepad1.dpad_right && speed < 1 && speedTimer.seconds() > 0.25) {
            speedTimer.reset();
            speed += 0.25;
        } else if (gamepad1.dpad_left && speed > 0.25 && speedTimer.seconds() > 0.25) {
            speedTimer.reset();
            speed -= 0.25;
        }

        if (gamepad2.right_stick_y <= 0) {
            shoulder.listen();
        } else {
            shoulderV0.listen();
        }
        elbow.listen();
        intake.listen();
        viper.listen();
        claw.listen();

        shoulderV0.sendTelemetry();
        intake.sendTelemetry();
        claw.sendTelemetry();

        telemetry.addData("Speed:", speed);
        updateTelemetry(telemetry);

    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){    }
    @Override
    public void stop(){}

}
