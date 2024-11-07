package org.firstinspires.ftc.teamcode;

// IMPORT SUBSYSTEMS

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commands.CommandMaster;
import org.firstinspires.ftc.teamcode.roadrunner.KalmanDrive;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.extension.Extension;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.util.hardware.Component;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;

public class Robot {

    // SUBSYSTEM DECLARATIONS
    public Component[] components;
    public KalmanDrive drive;
    public CVMaster cv;
    public Lift lift;
    public Extension extension;
    public Arm arm;
    public Claw claw;

    public CommandMaster commands;
    public HardwareMap hardwareMap;

    // STATE VARS
    boolean auton;
    boolean intaking = false;
    Levels state = Levels.INIT;
    Gamepiece mode = Gamepiece.SAMPLE;
    SampleColors targetColor = SampleColors.YELLOW;

    Motor backLeft;
    Motor backRight;
    Motor frontLeft;
    Motor frontRight;


    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        this.drive = new KalmanDrive(map, new Pose2d(0,0,0), limelight);

//        this.cv = new CVMaster(map);
        this.components = new Component[]{
                new Motor(3, "leftBack", map, true),          //0 left odometer
                new Motor(2, "rightBack", map, false),        //1 right odometer
                new Motor(1, "leftFront", map, true),         //2 middle odometer
                new Motor(0, "rightFront", map, false),       //3

                new Motor(0, "lift1", map, false),            //4
                new Motor(1, "lift2", map, false),            //5

                new StepperServo(0, "ext1", map),                    //6
                new StepperServo(1, "ext2", map),                    //7

                new StepperServo(0, "arm", map),                     //8

                new StepperServo(0, "elbow", map),                   //9
                new ContinuousServo(1, "intake1", map),              //10
                new ContinuousServo(2, "intake2", map)               //11
        };

        VoltageSensor voltageSensor = map.voltageSensor.iterator().next();
        RevColorSensorV3 colorSensor = map.get(RevColorSensorV3.class, "colorSensor");

        // INIT SUBSYSTEMS

        this.lift = new Lift((Motor) components[4], (Motor) components[5], voltageSensor);
        this.extension = new Extension((StepperServo) components[6], (StepperServo) components[7]);
        this.arm = new Arm((StepperServo) components[8], (StepperServo) components[9]);
        this.claw = new Claw((ContinuousServo) components[10], (ContinuousServo) components[11], colorSensor);

        this.commands = new CommandMaster(this);
        this.cv = new CVMaster(limelight, hardwareMap.get(WebcamName.class, "Webcam 1"));
        this.hardwareMap = map;

        backLeft = (Motor) components[0];
        backRight = (Motor) components[1];
        frontLeft = (Motor) components[2];
        frontRight = (Motor) components[3];
    }

    public Pose2d calculateRobotTargetCapturePose(Pose3D target, Pose2d robotPose) {
        return new Pose2d(target.getPosition().x, (robotPose.position.y), robotPose.heading.toDouble());
    }

    public void toggleGamepiece() {
        if (mode == Gamepiece.SAMPLE) {
            mode = Gamepiece.SPECIMEN;
        } else {
            mode = Gamepiece.SAMPLE;
        }
    }

    public void toggleGamepiece(Gamepiece p) {
        mode = p;
    }

    public void toggleGamepieceColor(AllianceColor allianceColor) {
        if (targetColor == SampleColors.BLUE || targetColor == SampleColors.RED) {
            targetColor = SampleColors.YELLOW;
        } else {
            if (allianceColor == AllianceColor.RED) {
                targetColor = SampleColors.RED;
            } else {
                targetColor = SampleColors.BLUE;
            }
        }
    }

    // INTAKE PRESETS

    public void intakePreset() {
        lift.runToPreset(Levels.INTAKE);
        arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
        extension.runToPreset(Levels.INTAKE);

        new Thread(() -> {
            sleep(3000);
            arm.runToPreset(Levels.INTAKE);
            claw.startIntake();
            intaking = true;
            state = Levels.INTAKE;
        }).start();
    }

    public void intakePreset(double extTicks) {
        lift.runToPreset(Levels.INTAKE);
        arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
        //TODO: CONVERT FROM INCHES TO TICKS
        extension.runToPosition((float) extTicks);

        new Thread(() -> {
            sleep(3000);
            arm.runToPreset(Levels.INTAKE);
            claw.startIntake();
            intaking = true;
            state = Levels.INTAKE;
        }).start();
    }

    public void intermediatePreset() {
        arm.runToPreset(Levels.INTERMEDIATE);
        extension.runToPreset(Levels.INTERMEDIATE);
        lift.runToPreset(Levels.INTERMEDIATE);
        state = Levels.INTERMEDIATE;
    }

    public void stopIntake() {
        intaking = false;
        claw.stopIntake();
        intermediatePreset();
    }

    public boolean autoStopIntakeUpdate(SampleColors... colors) {
        int r = claw.smartStopDetect(colors);
        switch (r) {
            case 0:
                return true;
                break;
            case 1:
                stopIntake();
                return false;
                break;
            case -1:
                claw.eject();
                return true;
                break;
        }
    }

    // DEPOSIT PRESETS
     public void lowBasket() {
        arm.runToPreset(Levels.LOW_BASKET);
        lift.runToPreset(Levels.LOW_BASKET);
        state = Levels.LOW_BASKET;
     }

    public void highBasket() {
        arm.runToPreset(Levels.HIGH_BASKET);
        lift.runToPreset(Levels.HIGH_BASKET);
        state = Levels.HIGH_BASKET;
    }

    public void lowRung() {
        arm.runToPreset(Levels.LOW_RUNG);
        lift.runToPreset(Levels.LOW_RUNG);
        state = Levels.LOW_RUNG;
    }

    public void highRung() {
        arm.runToPreset(Levels.HIGH_RUNG);
        lift.runToPreset(Levels.HIGH_RUNG);
        state = Levels.HIGH_RUNG;
    }

    public void preloadHighRung() {
        arm.runToPosition(0);
        lift.runToPosition(0);
        state = Levels.HIGH_RUNG;
    }


    public void teleDepositPreset() {
        if (mode == Gamepiece.SAMPLE) {
            highBasket();
        } else {
            highRung();
        }
    }

    // OUTTAKE

    public void outtakeSample() {
        claw.eject();
    }

    public void outtakeSpecimen() {
        lift.runToPosition(lift.getPos() - 10);
        new Thread(() -> {
            sleep(50);
            claw.eject();
        }).start();
    }

    public void outtakeSpecimenPreload() {
        lift.runToPosition(lift.getPos() - 10);
        new Thread(() -> {
            sleep(50);
            claw.eject();
            lift.runToPosition(0);

        }).start();
    }

    public void smartOuttake() {
        if (state == Levels.LOW_BASKET || state == Levels.HIGH_BASKET) {
            outtakeSample();
            intermediatePreset();
            state = Levels.INTERMEDIATE;
        } else if (state == Levels.LOW_RUNG || state == Levels.HIGH_RUNG) {
            outtakeSpecimen();
            intermediatePreset();
            state = Levels.INTERMEDIATE;
        }
    }

    //DRIVE
    public void setDrivePower(double x, double y, double rx) {
        double powerFrontLeft = y + x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackLeft = (y - x + rx) * -1;
        double powerBackRight = (y + x - rx) * -1;

        if (Math.abs(powerFrontLeft) > 1 || Math.abs(powerBackLeft) > 1 ||
                Math.abs(powerFrontRight) > 1 || Math.abs(powerBackRight) > 1) {
            // Find the largest power
            double max;
            max = Math.max(Math.abs(powerFrontLeft), Math.abs(powerBackLeft));
            max = Math.max(Math.abs(powerFrontRight), max);
            max = Math.max(Math.abs(powerBackRight), max);

            // Divide everything by max (it's positive so we don't need to worry
            // about signs)
            powerFrontLeft /= max;
            powerBackLeft /= max;
            powerFrontRight /= max;
            powerBackRight /= max;
        }

        frontLeft.setSpeed((float)powerFrontLeft);
        frontRight.setSpeed((float)powerFrontRight);
        backLeft.setSpeed(-(float)powerBackLeft);
        backRight.setSpeed(-(float)powerBackRight);
    }

    public void sleep(int millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public enum Gamepiece {
        SAMPLE,
        SPECIMEN
    }
}