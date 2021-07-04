package org.firstinspires.ftc.teamcode.drive.SampleAutonPaths;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

//import org.firstinspires.ftc.teamcode.drive.Hardware.Robot;
@TeleOp(name="TwoDrivers", group="Linear Opmode")
public class SampleDriveTele extends LinearOpMode implements Runnable{
    public static HardwareFile robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new HardwareFile(hardwareMap);
        robot.driveTrain=new SampleMecanumDrive(hardwareMap);
        waitForStart();
        while (opModeIsActive()) {
            run();
        }
    }

    double lxMult = 1;
    double lyMult = 1;
    double rxMult = 1;
    public void run() {
        robot.driveTrain.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * lyMult,
                        -gamepad1.left_stick_x * lxMult,
                        -gamepad1.right_stick_x * 0.92 * rxMult
                )
        );
        setMultiplier();
        if(gamepad2.right_bumper){
            robot.intakeL.setPower(-1);
            robot.intakeR.setPower(-1);
            robot.in1.setPower(1);
            robot.in2.setPower(1);
        }else{
            robot.intakeL.setPower(0);
            robot.intakeR.setPower(0);
            robot.in1.setPower(0);
            robot.in2.setPower(0);
        }
        if(gamepad2.y){
            robot.arm1.setPosition(0.93);
            robot.arm2.setPosition (0.07);
            sleep(500);
            robot.grabber.setPosition(0.63);
            robot.grabber2.setPosition(0.29);
        }else if(gamepad2.a){
            robot.grabber.setPosition(0.13);
            robot.grabber2.setPosition(0.83);
            sleep(500);
            robot. arm1.setPosition(0.1);
            robot.arm2.setPosition (0.88);
        }
        if(gamepad2.left_bumper){
            robot.shooter.setPower(1);
            sleep(500);
            for(int i=0;i<=3;++i){
                robot.magup();
                robot.magup();
                robot.slapper.setPosition(0.35);
                sleep(100);
                robot.slapper.setPosition(0.5);
                sleep(1500);
            }
            robot.shooter.setPower(0);
            robot.magdown();
        }
        robot.driveTrain.update();
    }

    private void setMultiplier() {
        if (gamepad1.left_trigger >= 0.3) {
            lxMult = 0.5;
            rxMult = 0.5;
            lyMult = 0.5;
        } else {
            lxMult = 1;
            rxMult = 1;
            lyMult = 1;
        }
        if (gamepad1.right_bumper) {
            lxMult = -lxMult;
            lyMult = -lyMult;
        }
    }
}
