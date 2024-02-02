package org.firstinspires.ftc.teamcode;

import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.RAINBOW_PARTY_PALETTE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

@Config
@TeleOp(name= "FluffyTeleOp", group = "robot") //double check group name
public class FluffyTeleOp extends OpMode {

    enum LauncherStates {INIT, FIRST_TAP_STARTED, FIRST_TAP_ENDED, DOUBLE_TAPPED, MOTOR_STARTS, TIMER_1, TIMER_2, PUSHER, MOTOR_ENDS}
    LauncherStates myLauncherState=LauncherStates.INIT;

    ElapsedTime doubleTapTimer=new ElapsedTime();
    final double shortTime=5000.0;
    final double catchupTime=2000.0;
    ElapsedTime hangerTimer = new ElapsedTime();
    final double hangerWait = 90000;
    public static String fingerPosition = "Init";

    public static double position = 0;
    boolean isGameStarted = false;

    public double forward;
    public double strafe;
    public double turn;

    public static double THRESHOLD = .15;

    TeleFluffy myBot;

    @Override
    public void init() {

        myBot = new TeleFluffy(this);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void loop(){
        if(!isGameStarted){
            hangerTimer.reset();
            isGameStarted = true;
        }
        setTeleOpDrive();
        setTeleOpLift();
        setGrabberRot();
        setFingerPosition();
        setHanger();
        setLauncher();
        setLauncher();
        setSlowApproach();
    }

    public void setTeleOpDrive(){
        if (gamepad1.left_bumper){
             forward = -gamepad1.left_stick_y*.25;
             strafe = gamepad1.left_stick_x*.25;
             turn = .75*gamepad1.right_stick_x*.5;
        }else{
            forward = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = .75*gamepad1.right_stick_x;
        }
        myBot.setTeleOpDrive(forward, strafe, turn);
        myBot.reportDriveMotors();
    }
    public void setTeleOpLift(){
        double liftMotorPower = (-gamepad2.left_stick_y);
        //double liftMotorPower = myBot.trimMotorPower(liftMotorPowerOriginal);
        if(liftMotorPower>0){
            myBot.setLiftUp(liftMotorPower);
            position = myBot.getLiftMotorPosition();
            telemetry.addLine("up");
        }else if(liftMotorPower<0){
            myBot.setLiftDown(liftMotorPower);
            position = myBot.getLiftMotorPosition();
            telemetry.addLine("down");
        }else myBot.setLiftStay(position);
        telemetry.addData("Lift position:", myBot.getLiftMotorPosition());
        telemetry.addData("Lift power:", liftMotorPower);
        telemetry.addData("Lift target:", myBot.getTargetPosition());
    }

    public void setGrabberRot(){
        if(gamepad2.dpad_up){
            myBot.raiseGrabber();
        }
        else if (gamepad2.dpad_down){
            myBot.lowerGrabber();
        }
    }


    public void setFingerPosition(){
        if (gamepad2.y){
            myBot.setFingerUp();
            fingerPosition = "Up";
        }
        if(gamepad2.a){
            myBot.setFingerDown();
            fingerPosition = "Down";
        }
        telemetry.addData("Finger position", fingerPosition);
    }

    public void setHanger(){
        //may need to make it a double click
        if (hangerTimer.milliseconds() >= hangerWait && gamepad1.right_bumper){
            myBot.releaseHanger();
        }
        if(gamepad1.right_trigger > THRESHOLD){
            myBot.moveHangerUp();
        }else if(gamepad1.left_trigger > THRESHOLD){
            myBot.moveHangerDown();
        }else{
            myBot.stopHanger();
        }
    }

    public void setLauncher() {
        switch (myLauncherState) {
            case INIT:
                if (gamepad2.left_bumper) {
                    myLauncherState = LauncherStates.FIRST_TAP_STARTED;
                }
                break;
            case FIRST_TAP_STARTED:
                if (!gamepad2.left_bumper) {
                    myLauncherState = LauncherStates.FIRST_TAP_ENDED;
                    doubleTapTimer.reset();
                }
                break;
            case FIRST_TAP_ENDED: {
                if (doubleTapTimer.milliseconds() <= shortTime) {
                    if (gamepad2.left_bumper) {
                        myLauncherState = LauncherStates.DOUBLE_TAPPED;
                    }
                } else {
                    myLauncherState = LauncherStates.INIT;
                }
                break;

                // if doubleTapTimer().milliseconds() > shortTime) { myLauncherState = LauncherStates.INIT; }
                // else {if (gamepad2.left_bumper) {myLauncherState = LauncherStates.DOUBLE_TAPPED;}}
            }
            case DOUBLE_TAPPED: {
                myLauncherState = LauncherStates.MOTOR_STARTS;
                myBot.setLeds(RAINBOW_PARTY_PALETTE);

                break;
            }
            case MOTOR_STARTS: {
                myBot.setDroneMotorSpeed();
                doubleTapTimer.reset();
                myLauncherState = LauncherStates.TIMER_1;

                break;
            }
            case TIMER_1: {
                if (doubleTapTimer.milliseconds() >= catchupTime) {
                    myLauncherState = LauncherStates.PUSHER;
                }

                break;
            }
            case PUSHER: {
                myBot.setDronePusherLaunch();
                doubleTapTimer.reset();
                RobotLog.i(String.format("Launch speed: %3.1f", myBot.droneMotor.getVelocity()));
                myLauncherState = LauncherStates.TIMER_2;

                break;
            }
            case TIMER_2: {
                if (doubleTapTimer.milliseconds() >= catchupTime) {
                    myLauncherState = LauncherStates.MOTOR_ENDS;
                }

                break;
            }
            case MOTOR_ENDS: {
                myBot.setDroneMotorZero();
                myBot.setDronePusherInit();
                myBot.setLeds(RAINBOW_PARTY_PALETTE);
                myLauncherState = LauncherStates.INIT;
                break;
            }
        }
        showState();

    }

    public void showState() {
        switch (myLauncherState) {
            case INIT: {
                telemetry.addLine("State: Init");
                break;
            }
            case FIRST_TAP_STARTED: {
                telemetry.addLine("State: First Tap Started");
                break;
            }
            case FIRST_TAP_ENDED: {
                telemetry.addLine("State: First Tap Ended");
                break;
            }
            case DOUBLE_TAPPED: {
                telemetry.addLine("State: Double Tapped");
                break;
            }
        }
    }

    public void setSlowApproach(){
        if (gamepad1.dpad_up){
            myBot.slowForward();
        }else if (gamepad1.dpad_down){
            myBot.slowBackward();
        }else if (gamepad1.dpad_right){
            myBot.slowRight();
        }else if (gamepad1.dpad_left){
            myBot.slowLeft();
        }

    }
}

