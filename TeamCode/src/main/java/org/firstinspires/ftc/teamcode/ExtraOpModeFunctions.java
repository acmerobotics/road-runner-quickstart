package org.firstinspires.ftc.teamcode;

//import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Collections;
import java.util.List;


@Config


public class ExtraOpModeFunctions
{
    public enum Signal {LEFT, MIDDLE, RIGHT}
    public enum RobotStartPosition {STRAIGHT, LEFT, RIGHT};
    public enum FieldSide {RED, BLUE}

    public FieldSide fs = FieldSide.BLUE;

    int numRed = 0;
    int numGreen = 0;
    int numBlue = 0;

    final boolean USING_WEBCAM = true;
    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
    final int RESOLUTION_WIDTH = 640;
    final int RESOLUTION_HEIGHT = 480;

    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime;


    public enum ElevatorPosition {COLLECT, GROUND, LOW, MIDDLE, HIGH, TWO, THREE, FOUR, FIVE}
    public ElevatorPosition elevatorPosition = ElevatorPosition.COLLECT;
    public enum ElbowPosition {EXTEND, RETRACT}
    public ElbowPosition elbowPosition = ElbowPosition.RETRACT;
    public static final double PI = 3.14159265;
    public int target = 0;

    //private VuforiaLocalizer vuforia = null;
    WebcamName webcamName = null;

    public Servo upperClaw;
    public Servo lowerClaw;
    public Servo elbow;
    public LinearOpMode localLop = null;

    public DcMotorEx elevatorLeft;
    public DcMotorEx elevatorRight;
    public DcMotorEx intake;
    public DcMotorEx lift;
    public TouchSensor elevatorLimit;

    public RevColorSensorV3 colorSensor;
    public ColorRangeSensor testColorSensor;

    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern pattern;


    public HardwareMap hm = null;
    public LinearOpMode lop = null;

    public ExtraOpModeFunctions(HardwareMap hardwareMap, LinearOpMode linearOpMode, FieldSide fieldSide)
    {
        hm = hardwareMap;
        lop = linearOpMode;
        fs = fieldSide;
        upperClaw = hardwareMap.get(Servo.class, "upperClaw");
        lowerClaw = hardwareMap.get(Servo.class, "lowerClaw");
        elbow = hardwareMap.get(Servo.class, "elbow");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        lift = hardwareMap.get(DcMotorEx.class, "lift");


        elevatorLeft = hardwareMap.get(DcMotorEx.class, "elevatorLeft");
        elevatorRight = hardwareMap.get(DcMotorEx.class, "elevatorRight");

        //webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        localLop = lop;

        intake.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        //intake.setTargetPosition(0);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setDirection(DcMotorEx.Direction.FORWARD);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        //lift.setTargetPosition(0);
        //lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(10.0, 0.05, 0.0, 0.0);
        //PIDFCoefficients pidfCoefficients = new PIDFCoefficients(5.0, 0.0, 0.0, 0.0);

        elevatorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        //elevatorLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);
        //elevatorLeft.setPositionPIDFCoefficients(5.0);
        elevatorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elevatorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorLeft.setTargetPosition(0);
        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        elevatorRight.setDirection(DcMotorEx.Direction.REVERSE);
        //elevatorRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_TO_POSITION, pidfCoefficients);
        //elevatorRight.setPositionPIDFCoefficients(5.0);
        elevatorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        elevatorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setTargetPosition(0);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        elevatorLimit = hardwareMap.get(TouchSensor.class, "elevatorLimit");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE;
        blinkinLedDriver.setPattern(pattern);
        displayPattern();

        elbowRetract();
        lowerClawRelease();
        upperClawRelease();

        initTfod();

    }

    public void initElevator()
    {
        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        elevatorLeft.setPower(0.2);
        elevatorRight.setPower(0.2);
        localLop.sleep(600);
        elevatorLeft.setPower(0);
        elevatorRight.setPower(0);


        localLop.telemetry.addData("Limit ", elevatorLimit.getValue());
        //localLop.telemetry.update();

        localLop.sleep(100);

        elevatorLeft.setPower(-0.1);
        elevatorRight.setPower(-0.1);

        while(!elevatorLimit.isPressed())
        {
            ;
        }

        elevatorLeft.setPower(0);
        elevatorRight.setPower(0);

        elevatorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        elevatorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        localLop.sleep(250);

        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        elevatorLeft.setTargetPosition(20);
        elevatorRight.setTargetPosition(20);

        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);

        localLop.telemetry.addLine("Elevator Initialized!");
        localLop.telemetry.update();

    }

    public void upperClawRelease()
    {
        upperClaw.setPosition(0);
    }

    // Red
    public void upperClawGrab()
    {
        upperClaw.setPosition(0.7);
    }

    // No color
    public void lowerClawRelease()
    {
        lowerClaw.setPosition(1);
    }

    public void lowerClawGrab()
    {
        lowerClaw.setPosition(0);
    }

    public void elbowExtend()
    {
        elbow.setPosition(0.37);
        elbowPosition = ElbowPosition.EXTEND;
    }

    public void elbowRetract()
    {
        elbow.setPosition(0.523);
        elbowPosition = ElbowPosition.RETRACT;
    }

    public void clawMove (int distance)
    {
        upperClaw.setPosition(upperClaw.getPosition() + distance);
    }

    public void elbowMove (int distance)
    {
        elbow.setPosition(elbow.getPosition() + distance);
    }

    //public void intakeOn()
    //{
        //intake.setPower(1);
    //}

    public void intakeForward() {
        intake.setPower(1);
    }
    public void intakeReverse() {
        intake.setPower(-1);
    }

    public void intakeOff()
    {
        intake.setPower(0);
    }

    public void liftExtend() {
        //lift.setTargetPosition(lift.getTargetPosition()+100);
        lift.setPower(1.0);
    }

    public void liftRetract() {
        //lift.setTargetPosition(lift.getTargetPosition()-100);
        lift.setPower(-1.0);
    }

    public void liftOff() {
        lift.setPower(0);
    }

    public void setElevatorPosition(int target)
    {
        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);
        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);
    }

    public enum ArmState {POSITIONIN, TOPOSITIONIN, POSITIONOUT, TOPOSITIONOUT}
    ArmState armState = ArmState.POSITIONIN;
    public enum ArmStateMachineAction {IDLE, XPRESSED, BPRESSED}

    ElapsedTime armStateTimer = new ElapsedTime(MILLISECONDS);
    public void armStateMachine(ArmStateMachineAction armStateMachineAction)
    {
        switch (armState)
        {
            case POSITIONIN:
                if(armStateMachineAction == ArmStateMachineAction.BPRESSED)
                {
                    step = 1;
                    armState = ArmState.TOPOSITIONOUT;
                }
                break;

            case TOPOSITIONIN:
                //Position in state machine
                switch (step)
                {
                    case 1:
                        lowerClawRelease();
                        upperClawRelease();
                        armStateTimer.reset();
                        step = 2;
                        break;
                    case 2:
                        if (armStateTimer.time() > 100)
                        {
                            step = 3;
                        }
                        break;
                    case 3:
                        elbowRetract();
                        armStateTimer.reset();
                        step = 4;
                        break;

                    case 4:
                        if (armStateTimer.time() > 1200)
                        {
                            step = 5;
                        }
                        break;

                    case 5:
                        target = 30;
                        //elevatorPosition = 0;
                        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        elevatorLeft.setTargetPosition(target);
                        elevatorRight.setTargetPosition(target);

                        elevatorLeft.setPower(0.5);
                        elevatorRight.setPower(0.5);

                        step = 6;
                        //armMoving = false;
                        break;

                    case 6:
                        if (elevatorLeft.getCurrentPosition() < 35)
                        {
                            elevatorLeft.setPower(0.0);
                            elevatorRight.setPower(0.0);
                            armState = ArmState.POSITIONIN;
                        }
                        break;

                }
                break;

            case POSITIONOUT:
                if(armStateMachineAction == ArmStateMachineAction.XPRESSED)
                {
                    step = 1;
                    armState = ArmState.TOPOSITIONIN;
                }
                break;

            case TOPOSITIONOUT:
                //position out state machine
                switch (step)
                {
                    case 1:
                        target = 0;
                        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        elevatorLeft.setTargetPosition(target);
                        elevatorRight.setTargetPosition(target);
                        elevatorLeft.setPower(1.0);
                        elevatorRight.setPower(1.0);
                        armStateTimer.reset();

                        step = 2;
                        break;
                    case 2:
                        if (armStateTimer.time() > 500)
                        {
                            step = 3;
                        }
                        break;
                    case 3:
                        lowerClawGrab();
                        upperClawGrab();
                        armStateTimer.reset();
                        step = 4;
                        break;

                    case 4:
                        if (armStateTimer.time() > 750)
                        {
                            step = 5;
                        }
                        break;

                    case 5:
                        target = 1000;

                        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                        elevatorLeft.setTargetPosition(target);
                        elevatorRight.setTargetPosition(target);

                        elevatorLeft.setPower(1.0);
                        elevatorRight.setPower(1.0);
                        armStateTimer.reset();
                        step = 6;
                        break;

                    case 6:
                        if (armStateTimer.time() > 800)
                        {
                            step = 7;
                        }
                        break;

                    case 7:
                        elbowExtend();
                        armState = ArmState.POSITIONOUT;
                        //armMoving = false;
                        break;

                }
                break;
        }


    }

    public boolean armMoving = false;
    public int step = 0;

    public class ClawReleaseAction implements Action
    {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            lowerClawRelease();
            upperClawRelease();
            return false;
        }
    }

    public Action clawReleaseAction()
    {
        return new ClawReleaseAction();
    }

    public class ClawGrabAction implements Action
    {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            lowerClawGrab();
            upperClawGrab();
            return false;
        }
    }
    public Action clawGrabAction()
    {
        return new ClawGrabAction();
    }

    ////
    public class ElbowRetractAction implements Action
    {
        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            elbowRetract();
            return false;
        }
    }
    public Action elbowRetractAction()
    {
        return new ElbowRetractAction();
    }

    ////
    public class ElbowExtendAction implements Action
    {
        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            elbowExtend();
            return false;
        }
    }
    public Action elbowExtendAction()
    {
        return new ElbowExtendAction();
    }

    public class SetElevatorPosition15Action implements Action
    {
        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            setElevatorPosition(15);
            return false;
        }
    }
    public Action setElevatorPosition15Action()
    {
        return new SetElevatorPosition15Action();
    }


/*
    public class PositionInAction implements Action
    {
        private boolean initialized = false;
        //int step = 1;
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            boolean done = true;

            if (!initialized)
            {
                timer.reset();
                step = 1;
                initialized = true;
                armMoving = true;
            }

            //Position in state machine
            switch (step)
            {
                case 1:
                    lowerClawRelease();
                    upperClawRelease();
                    step = 2;
                    break;
                case 2:
                    if (timer.time() > 100)
                    {
                        step = 3;
                    }
                    break;
                case 3:
                    elbowRetract();
                    timer.reset();
                    step = 4;
                    break;

                case 4:
                    if (timer.time() > 1200)
                    {
                        step = 5;
                    }
                    break;

                case 5:
                    target = 15;
                    //elevatorPosition = 0;
                    elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    elevatorLeft.setTargetPosition(target);
                    elevatorRight.setTargetPosition(target);

                    elevatorLeft.setPower(0.5);
                    elevatorRight.setPower(0.5);

                    done = false;
                    armMoving = false;
                    break;

            }

            return done;
        }
    }

    public Action positionInAction()
    {
        return new PositionInAction();
    }
*/
    /*
    public void positionIn()
    {
        lowerClawRelease();
        upperClawRelease();

        localLop.sleep(800);

        elbowRetract();

        localLop.sleep(1200);

        target = 15;
        //elevatorPosition = 0;
        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);

        elevatorLeft.setPower(0.5);
        elevatorRight.setPower(0.5);
    }

    public class PositionOutAction implements Action
    {
        private boolean initialized = false;
        //int step = 1;
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);

        @Override
        public boolean run(@NonNull TelemetryPacket packet)
        {
            boolean done = true;

            if (!initialized)
            {
                timer.reset();
                step = 1;
                armMoving = true;
                initialized = true;
            }

            //position out state machine
            switch (step)
            {
                case 1:
                    target = 0;
                    elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    elevatorLeft.setTargetPosition(target);
                    elevatorRight.setTargetPosition(target);
                    elevatorLeft.setPower(1.0);
                    elevatorRight.setPower(1.0);

                    step = 2;
                    break;
                case 2:
                    if (timer.time() > 500)
                    {
                        step = 3;
                    }
                        break;
                case 3:
                    lowerClawGrab();
                    upperClawGrab();
                    timer.reset();
                    step = 4;
                    break;

                case 4:
                    if (timer.time() > 750)
                    {
                        step = 5;
                    }
                    break;

                case 5:
                    target = 1000;

                    elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    elevatorLeft.setTargetPosition(target);
                    elevatorRight.setTargetPosition(target);

                    elevatorLeft.setPower(1.0);
                    elevatorRight.setPower(1.0);
                    timer.reset();
                    step = 6;
                    break;

                case 6:
                    if (timer.time() > 800)
                    {
                        step = 7;
                    }
                    break;

                case 7:
                    elbowExtend();
                    done = false;
                    armMoving = false;
                    break;

            }

            return done;
        }
    }

    public Action positionOutAction()
    {
        return new PositionOutAction();
    }

    public void positionOut()
    {
        target = 0;
        //elevatorPosition = 0;
        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);

        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);
        localLop.sleep(500);

        lowerClawGrab();
        upperClawGrab();
        localLop.sleep(750);

        target = 1000;

        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);

        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);

        localLop.sleep(800);
        elbowExtend();
    }
*/
    public void elevatorLow()
    {
        target = 1210;
        elevatorPosition = elevatorPosition.LOW;

        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);

        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);
    }

    public void elevatorMiddle()
    {
        target = 2020;
        elevatorPosition = elevatorPosition.MIDDLE;

        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);

        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);
    }

    public void elevatorHigh()
    {
        target = 2820;
        elevatorPosition = elevatorPosition.HIGH;

        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);

        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);
    }


    public void elevatorTwo()
    {
        target = 130;
        elevatorPosition = elevatorPosition.TWO;

        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);

        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);
    }

    public void elevatorThree()
    {
        target = 247;
        elevatorPosition = elevatorPosition.THREE;

        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);

        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);
    }

    public void elevatorFour()
    {
        target = 344;
        elevatorPosition = elevatorPosition.FOUR;

        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);

        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);
    }

    public void elevatorFive()
    {
        target = 443;
        elevatorPosition = elevatorPosition.FIVE;

        elevatorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevatorLeft.setTargetPosition(target);
        elevatorRight.setTargetPosition(target);

        elevatorLeft.setPower(1.0);
        elevatorRight.setPower(1.0);
    }

    public double adjustAngleForDriverPosition(double angle, RobotStartPosition robotStartPosition)
    {
        switch (robotStartPosition)
        {
            case STRAIGHT:
                angle = angle + PI/2;
                if(angle > (PI))
                    angle = angle - (PI*2);
                break;
            case LEFT:
                angle = angle - PI/2;
                if(angle < (-PI))
                    angle = angle + (PI*2);
                break;
            case RIGHT:
                angle = angle + PI/2;
                if(angle > (PI))
                    angle = angle - (PI*2);
                break;
        }
        return angle;
    }

    protected void displayPattern()
    {
        blinkinLedDriver.setPattern(pattern);
    }

    public void setLeds(double time)
    {
        if (time < 30)
        {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_BEATS_PER_MINUTE;
        }
        else if (time >= 30 && time <= 32)
        {
            pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
        }
        else
        {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_2_COLOR_WAVES;
        }

        displayPattern();
    }
    // CAMERA LOGIC
    private TfodProcessor tfod;
    public VisionPortal visionPortal;
    private void initTfod()
    {
        if (fs == FieldSide.RED)
        {
            tfod = new TfodProcessor.Builder()
                    .setModelFileName("MarkerRed.tflite")
                    .setModelLabels(Collections.singletonList("Marker"))
                    .build();
        }
        else
        {
            tfod = new TfodProcessor.Builder()
                    .setModelFileName("MarkerBlue.tflite")
                    .setModelLabels(Collections.singletonList("BlueMarker"))
                    .build();
        }
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hm.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(tfod);
        visionPortal = builder.build();
    }
    public enum AutoStart {RL, RR, BL, BR}
    public Signal telemetryTfod(AutoStart as)
    {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        localLop.telemetry.addData("# Objects Detected", currentRecognitions.size());
        Signal position = Signal.RIGHT;
        // Step through the list of recognitions and display info for each one.
        if(currentRecognitions.size() == 0)
        {
            switch (as)
            {
                case BL:
                    break;
                case BR:
                    localLop.telemetry.addData("Left_BR", 0);
                    position = Signal.LEFT;
                    break;
                case RL:
                    localLop.telemetry.addData("Right_RL", 0);
                    position = Signal.RIGHT;
                    break;
                case RR:
                    localLop.telemetry.addData("Left_RR", 0);
                    position = Signal.LEFT;
                    break;

            }
        }
        else
        {
            Recognition recognition = currentRecognitions.get(0);

            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            switch (as)
            {
                case BL:
                    if (x <= 200) {
                        localLop.telemetry.addData("Left_BL", x);
                        position = Signal.LEFT;
                    }else if (x <= 500) {
                        localLop.telemetry.addData("Middle_BL",x);
                        position = Signal.MIDDLE;
                    } else {
                        localLop.telemetry.addData("Right_BL", x);
                        position = Signal.RIGHT;
                    }
                    break;
                case BR:
                    if (x <= 300) {
                        localLop.telemetry.addData("Middle_BR",x);
                        position = Signal.MIDDLE;
                    } else if (x <= 500) {
                        localLop.telemetry.addData("Right_BR", x);
                        position = Signal.RIGHT;
                    } else {
                        position = Signal.LEFT;
                    }
                    break;
                case RL:
                    if (x <= 200) {
                        localLop.telemetry.addData("Left_RL",x);
                        position = Signal.LEFT;
                    } else if (x <= 500) {
                        localLop.telemetry.addData("Middle_RL", x);
                        position = Signal.MIDDLE;
                    } else {
                        position = Signal.RIGHT;
                    }
                    break;
                case RR:
                    if (x <= 250) {
                        localLop.telemetry.addData("Middle_RR",x);
                        position = Signal.MIDDLE;
                    } else if (x >= 251) {
                        localLop.telemetry.addData("Right_RR", x);
                        position = Signal.RIGHT;
                    }
                    break;

            }

            //localLop.telemetry.addData(""," ");
            //localLop.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            localLop.telemetry.addData("- Position", "%.0f / %.0f", x, y);
            //localLop.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            //localLop.telemetry.update();
        }   // end for() loop
        return (position);
    }





}