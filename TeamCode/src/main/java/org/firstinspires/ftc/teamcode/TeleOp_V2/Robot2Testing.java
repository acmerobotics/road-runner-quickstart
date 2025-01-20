package org.firstinspires.ftc.teamcode.TeleOp_V2;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp(name = "Robot2Testing")
public class Robot2Testing extends LinearOpMode{
    private SimpleServo Coax;
    private SimpleServo V4B;
    private SimpleServo ExtRight;
    private SimpleServo ExtLeft;

    private SimpleServo OuttakeRight;
    private SimpleServo OuttakeLeft;


    private GamepadEx gamepad1Ex;    private MotorEx LiftRight;
    private MotorEx LiftLeft;
    private GamepadEx gamepad2Ex;

    @Override
    public void runOpMode() throws InterruptedException {
        Coax = new SimpleServo(hardwareMap, "Coax", 0.0, 1.0);
        V4B = new SimpleServo(hardwareMap, "V4B", 0.0, 1.0);

        ExtLeft = new SimpleServo(hardwareMap, "ExtL", 0.0, 1.0);
        ExtRight = new SimpleServo(hardwareMap, "ExtR", 0.0, 1.0);

        OuttakeRight = new SimpleServo(hardwareMap, "OR", 0.0, 1.0);
        OuttakeLeft = new SimpleServo(hardwareMap, "OL", 0.0, 1.0);

        LiftLeft = new MotorEx(hardwareMap, "LiftLeft");
        LiftLeft.setRunMode(Motor.RunMode.RawPower);
        LiftLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        LiftRight = new MotorEx(hardwareMap, "LiftRight");
        LiftRight.setRunMode(Motor.RunMode.RawPower);
        LiftRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        gamepad1Ex = new GamepadEx(gamepad1);
        gamepad2Ex = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2Ex.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
                Coax.setPosition(0);
            } else if (gamepad2Ex.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
                Coax.setPosition(1);
            }

            if (gamepad2Ex.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1){
                V4B.setPosition(0);
            } else if (gamepad2Ex.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1){
                V4B.setPosition(1);
            }

            if (gamepad2Ex.getLeftY() > 0.1){ // Slides UP
                LiftLeft.set(-1);
                LiftRight.set(1);
            } else if (gamepad2Ex.getLeftY() < -0.1) { // Slides Down
                LiftLeft.set(0.4);
                LiftRight.set(-0.4);
            } else { // Hold Slide Position
                LiftLeft.set(0.075);
                LiftRight.set(-0.075);
            }

            if (gamepad2Ex.getRightY() > 0.1){ // Extension Out
                ExtLeft.setPosition(ExtLeft.getPosition() - 0.05);
                ExtRight.setPosition(ExtRight.getPosition() + 0.05);
            } else if (gamepad2Ex.getRightY() < -0.1) { // Extension In
                ExtLeft.setPosition(ExtLeft.getPosition() + 0.05);
                ExtRight.setPosition(ExtRight.getPosition() - 0.05);
            }

            if(gamepad2Ex.getButton(GamepadKeys.Button.DPAD_DOWN)){
                OuttakeLeft.setPosition(1);
                OuttakeRight.setPosition(0);
            } else if (gamepad2Ex.getButton(GamepadKeys.Button.DPAD_UP)){
                OuttakeLeft.setPosition(0);
                OuttakeRight.setPosition(1);
            }
        }
    }
}
