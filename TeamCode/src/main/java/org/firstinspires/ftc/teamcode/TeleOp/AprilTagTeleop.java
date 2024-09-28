package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Common.Arm;
import org.firstinspires.ftc.teamcode.Common.Claw;
import org.firstinspires.ftc.teamcode.Common.Constants;
import org.firstinspires.ftc.teamcode.Common.Drone;
import org.firstinspires.ftc.teamcode.Common.Hanging;
import org.firstinspires.ftc.teamcode.Common.YServo;
import org.firstinspires.ftc.teamcode.auto.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.auto.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="TeleOp with April Tag")

public class AprilTagTeleop extends LinearOpMode {
    static GamepadEx driverGamepad;
    static GamepadEx armGamepad;
    private ToggleButtonReader toggleClaw;
    private ToggleButtonReader toggleClaw1;
    private ToggleButtonReader toggleClaw2;
    public double factor = 1;
    public double strafeFactor = 1;
    public double turnFactor = 0.7;
    private Arm arm;
    private YServo Yservo;
    private Claw claw;
    private Drone drone;
    private Hanging hanging;
    private SelectTag tagMenu;
    private AprilTagLocalization aprilTag;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        driverGamepad = new GamepadEx(gamepad1);
        armGamepad = new GamepadEx(gamepad2);

        initMotors();

        tagMenu = new SelectTag(gamepad1, telemetry, isStopRequested());

        aprilTag = new AprilTagLocalization(hardwareMap, telemetry);
        aprilTag.AprilTagHardwareStart(isStopRequested());
        aprilTag.setDesiredDistance(4);

        toggleClaw = new ToggleButtonReader(armGamepad, GamepadKeys.Button.B);
        toggleClaw1 = new ToggleButtonReader(armGamepad, GamepadKeys.Button.DPAD_LEFT);
        toggleClaw2 = new ToggleButtonReader(armGamepad, GamepadKeys.Button.DPAD_RIGHT);

        waitForStart();

        while (opModeIsActive()) {

            toggleClaw.readValue();
            toggleClaw1.readValue();
            toggleClaw2.readValue();

            if (arm.getPosition() > Constants.ArmUpTicks - 500){
                strafeFactor = 0.75;
            } else {
                strafeFactor = 1;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                        -factor * driverGamepad.getLeftY(),
                        -strafeFactor * driverGamepad.getLeftX(),
                        -turnFactor * driverGamepad.getRightX()
                    )
            );

            gamepad1Functions();
            gamepad2Functions();
            drive.update();

            telemetry.addData("x", drive.getPoseEstimate().getX());
            telemetry.addData("y", drive.getPoseEstimate().getY());
            telemetry.addData("heading", drive.getPoseEstimate().getHeading());

            telemetry.update();
        }
    }

    private void initMotors(){
        arm = new Arm(hardwareMap);
        arm.resetPos();

        drone = new Drone(hardwareMap);
        drone.resetDrone();

        hanging = new Hanging(hardwareMap);
        hanging.hangingDown();

        claw = new Claw(hardwareMap);
        claw.ClawClosed();

        Yservo = new YServo(hardwareMap);
        Yservo.YServoUp();
    }

    private void gamepad1Functions() throws InterruptedException{
        if (driverGamepad.getButton(GamepadKeys.Button.B)){
            drone.flyDrone();
        }

        if (driverGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)){
            factor = 0.75;
            turnFactor = 0.6;
        } else{
            factor = 1;
            turnFactor = 0.7;
        }

        tagMenu.ShowMenu(isStopRequested());
        aprilTag.setDesiredTagId(tagMenu.getTag());

        if (driverGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)){
            try {
                aprilTag.aprilTagDetection(isStopRequested());
            } catch (InterruptedException e) {
                sleep(10);
            }
        }
    }
    private void gamepad2Functions(){
        if (armGamepad.getButton(GamepadKeys.Button.LEFT_BUMPER)) {
            arm.ArmUp(Constants.armSpeedUp);
        } else if (armGamepad.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            arm.ArmDown(Constants.armSpeedDown);
        } else if (armGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0){
            arm.ArmChangePos(Math.round((float) (Constants.armIncrement * armGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))), Constants.armSpeedUp);
        } else if (armGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) != 0){
            arm.ArmChangePos((Math.round((float) (-Constants.armIncrement * armGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)))), Constants.armSpeedDown);
        }

        if(armGamepad.getButton(GamepadKeys.Button.Y)) {
            Yservo.YServoDown();
        } else if(armGamepad.getButton(GamepadKeys.Button.A)) {
            Yservo.YServoUp();
        }

        if(toggleClaw.getState()){
            claw.ClawOpen();
        } else if(toggleClaw1.getState()) {
            claw.Claw1Open();
        } else if(toggleClaw2.getState()) {
            claw.Claw2Open();
        } else{
            claw.ClawClosed();
        }

        if(armGamepad.getButton(GamepadKeys.Button.B)){
            hanging.hangingMiddle();
        } else if(armGamepad.getButton(GamepadKeys.Button.DPAD_DOWN)){
            hanging.hangingDown();
        } else if(armGamepad.getButton(GamepadKeys.Button.DPAD_UP)){
            hanging.hangingUp();
        }
    }
}
