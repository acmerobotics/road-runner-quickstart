package org.firstinspires.ftc.teamcode.opmodes;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.tidev2.Claw;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Elbow;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Intake;
import org.firstinspires.ftc.teamcode.hardware.tidev2.automation.BucketOperatorFSM;
import org.firstinspires.ftc.teamcode.hardware.tidev2.Shoulder;
import org.firstinspires.ftc.teamcode.hardware.tidev2.automation.SubOperatorFSM;
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
    Elbow elbow = new Elbow(this);
    Intake intake = new Intake(this);
    Viper viper = new Viper(this);
    Claw claw = new Claw(this);

    MecanumDrive drive;

    Pose2d poseEstimate;
    double speed;

    ElapsedTime speedTimer = new ElapsedTime();

    ElapsedTime resetTimer = new ElapsedTime();

    BucketOperatorFSM bucketOperatorFSM;
    SubOperatorFSM subOperatorFSM;


    @Override
    public void init() {
        max = 1;

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back_drive");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front_drive");

        shoulder.init();
        elbow.init();
        intake.init();
        viper.init();
        claw.init();
        speed = 1;

        bucketOperatorFSM = new BucketOperatorFSM(gamepad2, shoulder, viper, elbow, claw, intake);
        subOperatorFSM = new SubOperatorFSM(gamepad2, shoulder, viper, elbow, claw, intake);

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


        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            ;
        } else if (gamepad1.left_bumper) {
            leftFront.setPower(-max);
            leftBack.setPower(max);
            rightFront.setPower(max);
            rightBack.setPower(-max);
        } else if (gamepad1.right_bumper) {
            leftFront.setPower(max);
            leftBack.setPower(-max);
            rightFront.setPower(-max);
            rightBack.setPower(max);
        }

        if (gamepad1.dpad_right && speed < 1 && speedTimer.seconds() > 0.25) {
            speedTimer.reset();
            speed += 0.25;
        } else if (gamepad1.dpad_left && speed > 0.25 && speedTimer.seconds() > 0.25) {
            speedTimer.reset();
            speed -= 0.25;
        }

        bucketOperatorFSM.listen();
        subOperatorFSM.listen();
        shoulder.listen();
        shoulder.sendTelemetry();

        elbow.listen();
        intake.listen();
        viper.listen();
        claw.listen();

        elbow.sendTelemetry();
        intake.sendTelemetry();
        claw.sendTelemetry();
        viper.sendTelemetry();

        viper.passShoulderDeg(shoulder.toDegrees(shoulder.getCurrentPosition()));

        updateTelemetry(telemetry);

    }

    @Override
    public void init_loop(){}
    @Override
    public void start(){    }
    @Override
    public void stop(){}

}
