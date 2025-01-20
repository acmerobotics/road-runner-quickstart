package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.Extension;
import org.firstinspires.ftc.teamcode.Common.Lift;

@TeleOp(name= "Driver TeleOp")
public class PracticeTeleop2 extends LinearOpMode {
    static MecanumDrive drive;
    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;
    static RobotNew robot;
    static Extension extension;
    static Lift slides;
    long startTime;

    private void HardwareStart() {
        robot = new RobotNew();
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        // Init Claw Closed, Four Bar Completely Up, and Rotation at straight

    }
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareStart();
        waitForStart();

        while (opModeIsActive()) {
            long curTime = System.currentTimeMillis();
            drive.driveRobotCentric(
                    gamepad1Ex.getLeftX(),
                    gamepad1Ex.getLeftY(),
                    gamepad1Ex.getRightX(),
                    true
            );



        }
    }
}