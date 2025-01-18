package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.Extension;
import org.firstinspires.ftc.teamcode.Common.Lift;

@TeleOp(name = "Driver Teleop")
public class Teleop extends LinearOpMode {
    static MecanumDrive drive;
    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;
    static RobotNew robot;
    static Extension extension;
    static Lift slides;
    long startTime;

    private  void HardwareStart(){
        robot = new RobotNew();
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        // Init Claw Closed, Four Bar Completely Up, and Rotation at straight
        robot.ClawGrip.setPosition(0);
        robot.FourBarLeft.setPosition(1);
        robot.ClawRotation.setPosition(1);
        startTime = System.currentTimeMillis();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareStart();
        waitForStart();

        while(opModeIsActive()) {
            long curTime = System.currentTimeMillis();
            drive.driveRobotCentric(
                    gamepad1Ex.getLeftX(),
                    gamepad1Ex.getLeftY(),
                    gamepad1Ex.getRightX(),
                    true
            );

            if (gamepad2Ex.getLeftY() > 0.1){ // Slides UP
                robot.SlideLeft.set(1);
                robot.SlideRight.set(-1);
            } else if (gamepad2Ex.getLeftY() < -0.1) { // Slides Down
                robot.SlideLeft.set(-0.4);
                robot.SlideRight.set(0.4);
            } else { // Hold Slide Position
                robot.SlideLeft.set(0.15);
                robot.SlideRight.set(-0.15);
            }

            telemetry.addData("Slide Ticks Left", robot.SlideLeft.getCurrentPosition());
            telemetry.addData("Slide Ticks Right", robot.SlideRight.getCurrentPosition());
            telemetry.addData("Millis Since Start",  curTime - startTime);
            telemetry.update();

            if (gamepad2Ex.getRightY() > 0.1){ // Extension Out
                robot.ExtensionRight.set(1);
                robot.ExtensionLeft.set(-1);
            } else if (gamepad2Ex.getRightY() < -0.1) { // Extension In
                robot.ExtensionRight.set(-1);
                robot.ExtensionLeft.set(1);
            } else {
                robot.ExtensionRight.set(0);
                robot.ExtensionLeft.set(0);
            }

            if (gamepad2Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1){
                robot.FourBarLeft.setPosition(0.5);
            } else{
                robot.FourBarLeft.setPosition(1);
            }

            if (gamepad2Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1){
                robot.ClawGrip.setPosition(0.8);

            } else{
                robot.ClawGrip.setPosition(0);
            }

            if (gamepad2Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
                robot.ClawRotation.setPosition(1);
            }

            if (gamepad2Ex.getButton(GamepadKeys.Button.LEFT_BUMPER)){
                robot.ClawRotation.setPosition(0.5);
            }
        }
    }
}
