package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp V2")
public class TeleOpV2  extends LinearOpMode{
    static MecanumDrive drive;
    static GamepadEx gamepad1Ex;
    static GamepadEx gamepad2Ex;
    static RobotV2 robot;
    static double extpos = 0;

    private void HardwareStart(){
        robot = new RobotV2();
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot.FrontLeft, robot.FrontRight, robot.BackLeft, robot.BackRight);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        // Init Actions
        extpos = 0;

    }


    @Override
    public void runOpMode() throws InterruptedException {
        HardwareStart();
        waitForStart();

        while (opModeIsActive()) {
            drive.driveRobotCentric(
                    gamepad1Ex.getLeftX(),
                    gamepad1Ex.getLeftY(),
                    gamepad1Ex.getRightX(),
                    true
            );

            if (gamepad2Ex.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                robot.Coax.setPosition(0);
            } else if (gamepad2Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                robot.Coax.setPosition(1);
            }

            if (gamepad2Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1){
                robot.V4B.setPosition(0);
            } else if (gamepad2Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1){
                robot.V4B.setPosition(1);
            }

            if (gamepad2Ex.getLeftY() > 0.1){ // Slides UP
                robot.LiftLeft.set(-1);
                robot.LiftRight.set(1);
            } else if (gamepad2Ex.getLeftY() < -0.1) { // Slides Down
                robot.LiftLeft.set(0.7);
                robot.LiftRight.set(-0.7);
            } else { // Hold Slide Position
                robot.LiftLeft.set(0.02);
                robot.LiftRight.set(-0.02);
            }

            telemetry.addData("Extension Left Position", robot.ExtLeft.getPosition());


            if (gamepad2Ex.getRightY() > 0.1){ // Extension Out
                extpos = extpos + 0.05;
            } else if (gamepad2Ex.getRightY() < -0.1) { // Extension In
                extPos = extpos - 0.05;
            }

            if(gamepad2Ex.getButton(GamepadKeys.Button.DPAD_DOWN)){
                robot.OuttakeLeft.setPosition(1);
                robot.OuttakeRight.setPosition(0);
            } else if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_UP)){
                robot.OuttakeLeft.setPosition(0);
                robot.OuttakeRight.setPosition(1);
            }


            robot.ExtLeft.setPosition(extpos);
            robot.ExtRight.setPosition(1 - extpos);
            telemetry.addData("Extension Position Variable", extpos);
            telemetry.update();

        }
    }
}
