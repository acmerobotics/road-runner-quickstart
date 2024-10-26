package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Common.Extension;
import org.firstinspires.ftc.teamcode.Common.SlideGroup;
import org.firstinspires.ftc.teamcode.TeleOp.RobotNew;

@TeleOp(name = "Teleop-New")
public class TeleopNew extends LinearOpMode {
    static MecanumDrive drive;
    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;
    static RobotNew robot;
    static Extension extension;
    static SlideGroup slides;

    private  void HardwareStart(){
        robot = new RobotNew();
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);

        extension = new Extension(robot.Extension);
        slides = new SlideGroup(robot.Slides);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        HardwareStart();
        waitForStart();

        while(opModeIsActive()) {
            drive.driveRobotCentric(
                    gamepad1Ex.getLeftX(),
                    gamepad1Ex.getLeftY(),
                    gamepad1Ex.getRightX(),
                    true
            );

            if (gamepad2Ex.getButton(GamepadKeys.Button.A)) {
                extension.extend();
            }

            if (gamepad2Ex.getButton(GamepadKeys.Button.B)) {
                extension.retract();
            }

            if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_DOWN)) { // Slides Up
                robot.Slides.set(1);
            } else if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_UP)) { // Slides Down
                robot.Slides.set(-1);
            } else { // Hold Slide Position
                robot.Slides.set(-0.009375);
            }

            if (gamepad2Ex.getButton(GamepadKeys.Button.X)){
                slides.resetEncoder();
                slides.slidesToPosition(10);
            }
        }
    }
}
