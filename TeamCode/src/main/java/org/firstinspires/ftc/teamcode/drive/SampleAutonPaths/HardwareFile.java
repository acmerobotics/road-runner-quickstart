package org.firstinspires.ftc.teamcode.drive.SampleAutonPaths;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class HardwareFile {
    private LinearOpMode linearOpMode;
    public DcMotor intakeR, intakeL, shooter;

    public CRServo in1, in2;
    public Servo arm1, arm2;
    public Servo grabber, grabber2, slapper, tilter, shooterflap;
    public Servo rightIntakeHolder, leftIntakeHolder;
    HardwareMap map;
    public SampleMecanumDrive driveTrain;
    public static Pose2d robotPose = new Pose2d();

    public HardwareFile(HardwareMap imported) {
        //robotPose = new Pose2d(x - 70.4725, y - 70.4725, robotOrientation);
        construct(imported);
    }
    private void construct(HardwareMap imported){
        map = imported;
        intakeR = map.get(DcMotor.class, "intakeR");
        intakeL = map.get(DcMotor.class, "intakeL");
        shooter = map.get(DcMotor.class, "fw");
        in1 = map.crservo.get("in1");
        in2 = map.crservo.get("in2");
        in1.setDirection(DcMotorSimple.Direction.REVERSE);

        arm1 = map.get(Servo.class, "wobbleArm1");
        arm2 = map.get(Servo.class, "wobbleArm2");
        grabber = map.get(Servo.class, "wobbleGrabber1");
        grabber2 = map.get(Servo.class, "wobbleGrabber2");
        slapper = map.get(Servo.class, "mag");
        tilter = map.get(Servo.class, "tilt");
        shooterflap = map.get(Servo.class, "flap");
        leftIntakeHolder = map.get(Servo.class,"wallL");
        rightIntakeHolder = map.get(Servo.class,"wallR");
        intakeL.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public void wobbleArmUp() {
        arm1.setPosition(0.1);
        arm2.setPosition (0.88);
    }
    public void wobbleArmDown() {
        arm1.setPosition(0.93);
        arm2.setPosition (0.07);

    }
    public void wobbleArmVertical(){
        arm1.setPosition(0.5);
        arm2.setPosition (0.5);
    }
    public void grab(){
        grabber.setPosition(0.13);
        grabber2.setPosition(0.83);
    }
    public void release(){
        grabber.setPosition(0.63);
        grabber2.setPosition(0.29);
    }
    public void intake(double intakeSpeed){
        intakeL.setPower(-intakeSpeed);
        intakeR.setPower(-intakeSpeed);
        in1.setPower(intakeSpeed);
        in2.setPower(intakeSpeed);
    }

    public void shooter(double shooterpower){
        shooter.setPower(shooterpower);
    }
    public void magup(){
        tilter.setPosition(1);
    }
    public void magdown(){
        tilter.setPosition(0.5);
    }

}
