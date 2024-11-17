package org.firstinspires.ftc.teamcode;

// IMPORT SUBSYSTEMS

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.commands.CommandMaster;
import org.firstinspires.ftc.teamcode.roadrunner.KalmanDrive;
import org.firstinspires.ftc.teamcode.subsystems.claw.Claw;
import org.firstinspires.ftc.teamcode.subsystems.climb.ClimbWinch;
import org.firstinspires.ftc.teamcode.subsystems.extension.Extension;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.vision.CVMaster;
import org.firstinspires.ftc.teamcode.util.enums.AllianceColor;
import org.firstinspires.ftc.teamcode.util.hardware.BrushlandColorSensor;
import org.firstinspires.ftc.teamcode.util.hardware.Component;
import org.firstinspires.ftc.teamcode.util.hardware.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.enums.Levels;
import org.firstinspires.ftc.teamcode.util.hardware.Motor;
import org.firstinspires.ftc.teamcode.util.enums.SampleColors;
import org.firstinspires.ftc.teamcode.util.hardware.StepperServo;
import org.firstinspires.ftc.teamcode.util.misc.FullPose2d;

public class Robot {

    // SUBSYSTEM DECLARATIONS
    public Component[] components;
    public KalmanDrive drive;
    public CVMaster cv;
    public Lift lift;
    public Extension extension;
    public Arm arm;
    public Claw claw;
    public ClimbWinch climbWinch;

    public CommandMaster commands;
    public HardwareMap hardwareMap;

    // STATE VARS
    boolean auton;
    boolean intaking = false;
    public boolean l3ClimbOverride = false;
    public Levels state = Levels.INIT;
    Gamepiece mode = Gamepiece.SAMPLE;
    public SampleColors targetColor = SampleColors.YELLOW;

    Motor backLeft;
    Motor backRight;
    Motor frontLeft;
    Motor frontRight;

    public IMU imu;


    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        this.drive = new KalmanDrive(map, new Pose2d(0,0,0), limelight);

//        this.cv = new CVMaster(map);
        this.components = new Component[]{
                new Motor(3, "leftBack", map, true),                //0 left odometer
                new Motor(2, "rightBack", map, false),              //1 right odometer
                new Motor(1, "leftFront", map, true),               //2 middle odometer
                new Motor(0, "rightFront", map, false),             //3

                new Motor(0, "lift1", map, false),                  //4
                new Motor(1, "lift2", map, false),                  //5

                new StepperServo(0, "ext1", map, "ext1Encoder"),    //6
                new StepperServo(1, "ext2", map, "ext2Encoder"),    //7

                new StepperServo(0, "arm", map, "armEncoder"),      //8

                new StepperServo(0, "elbow", map, "elbowEncoder"),  //9
                new ContinuousServo(1, "intake1", map),                     //10
                new ContinuousServo(2, "intake2", map),                     //11

                new StepperServo(0, "climb1", map, "climb1Encoder"),//12
                new StepperServo(0, "climb2", map, "climb2Encoder") //13
        };

        VoltageSensor voltageSensor = map.voltageSensor.iterator().next();
//        RevColorSensorV3 colorSensor = map.get(RevColorSensorV3.class, "colorSensor");
        BrushlandColorSensor colorSensor = new BrushlandColorSensor(0, "color", map);

        // INIT SUBSYSTEMS

        this.lift = new Lift((Motor) components[4], (Motor) components[5], voltageSensor);
        this.extension = new Extension((StepperServo) components[6], (StepperServo) components[7]);
        this.arm = new Arm((StepperServo) components[8], (StepperServo) components[9]);
        this.claw = new Claw((ContinuousServo) components[10], (ContinuousServo) components[11], colorSensor);
        this.climbWinch = new ClimbWinch((StepperServo) components[12], (StepperServo) components[13]);

        this.imu = hardwareMap.get(IMU.class, "imu");

        this.commands = new CommandMaster(this);
        this.cv = new CVMaster(limelight, hardwareMap.get(WebcamName.class, "Webcam 1"));
        this.hardwareMap = map;

        backLeft = (Motor) components[0];
        backRight = (Motor) components[1];
        frontLeft = (Motor) components[2];
        frontRight = (Motor) components[3];
    }

    public Action generateTeleOpAutomatedIntake(Gamepad gamepad) {
        CVMaster.EOCVPipeline targetPipeline = CVMaster.EOCVPipeline.YELLOW_SAMPLE;
        switch (targetColor) {
            case RED:
                targetPipeline = CVMaster.EOCVPipeline.RED_SAMPLE;
                break;
            case BLUE:
                targetPipeline = CVMaster.EOCVPipeline.BLUE_SAMPLE;
                break;
        }
        cv.updatePotentialTargetList(targetPipeline, drive.pose);
        Pose3D targetPose = cv.findOptimalTarget(drive.pose);
        FullPose2d robotTargetPose = cv.calculateRobotFullPose(targetPose, drive.pose.position.x, drive.pose.position.y);
        if (robotTargetPose.intakeExtension > extension.MAXIMUM_EXTENSION) {
            // WHEN JUST TURNING ISNT ENOUGH FOR THE BOT TO REACH THE SAMPLE
            double normalHeading = normalizeRadians(drive.pose.heading.toDouble());
            if (normalHeading > ((3*Math.PI)/4) && normalHeading < ((5*Math.PI)/4)) {
                // ZONE I (Facing Right)
                robotTargetPose = cv.calculateRobotFullPose(targetPose, drive.pose.position.x, targetPose.getPosition().y);
            } else if (normalHeading > ((7*Math.PI)/4) || normalHeading < (Math.PI/4)) {
                // ZONE II (Facing Left)
                robotTargetPose = cv.calculateRobotFullPose(targetPose, drive.pose.position.x, targetPose.getPosition().y);
            } else if (normalHeading > ((5*Math.PI)/4) && normalHeading < ((7*Math.PI)/4)) {
                // ZONE III (Facing Down)
                robotTargetPose = cv.calculateRobotFullPose(targetPose, targetPose.getPosition().x, drive.pose.position.y);
            } else if (normalHeading > (Math.PI/4) && normalHeading < ((3*Math.PI)/4)) {
                // ZONE IV (Facing Up)
                robotTargetPose = cv.calculateRobotFullPose(targetPose, targetPose.getPosition().x, drive.pose.position.y);
            }
        }

        if (Math.sqrt(Math.pow((drive.pose.position.x - robotTargetPose.getRobotPose().position.x), 2) + Math.pow((drive.pose.position.y - robotTargetPose.getRobotPose().position.y), 2)) > 30
                || robotTargetPose.getRobotPose().position.x + 6 > 60 || robotTargetPose.getRobotPose().position.x - 6 < -60
                || robotTargetPose.getRobotPose().position.y - 7 < -60 || robotTargetPose.getRobotPose().position.y + 7 > 60) {
            // FAILSAFE TO STOP THE BOT FROM TRYING TO GO TO SOME CRAZY AHH LOCATION
            return new InstantAction(() -> gamepad.rumbleBlips(3));
        }

        Action path = drive.actionBuilder(drive.pose)
                .splineToLinearHeading(robotTargetPose.getRobotPose(), robotTargetPose.getRobotPose().heading)
                .build();
        Action pathBack = drive.actionBuilder(robotTargetPose.getRobotPose())
                .setReversed(true)
                .splineToLinearHeading(drive.pose, drive.pose.heading)
                .build();
        return new SequentialAction(
                path,
                intakePreset(robotTargetPose.intakeExtension, true),
                commands.stopIntake(targetColor),
                commands.waitForExtension(20),
                pathBack
        );
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

    public Action intakePreset(double extTicks, boolean action) {
        return new SequentialAction(
                new InstantAction(() -> {
                    lift.runToPreset(Levels.INTAKE);
                    arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
                    //TODO: CONVERT FROM INCHES TO TICKS
                    extension.runToPosition((float) extTicks);
                }),
                commands.waitForExtension((float) (extTicks - 20)),
                new InstantAction(() -> {
                    arm.runToPreset(Levels.INTAKE);
                    claw.startIntake();
                    intaking = true;
                    state = Levels.INTAKE;
                })
        );
    }

    public void autonObParkPreset() {
        lift.runToPreset(Levels.INTAKE);
        arm.runToPreset(Levels.INTAKE_INTERMEDIATE);
        extension.runToPosition(100);
    }

    public void intermediatePreset() {
        arm.runToPreset(Levels.INTERMEDIATE);
        extension.runToPreset(Levels.INTERMEDIATE);
        lift.runToPreset(Levels.INTERMEDIATE);
        state = Levels.INTERMEDIATE;
    }

    public void scanForTargetsPreset() {
        arm.runToPreset(Levels.INTERMEDIATE);
        extension.runToPreset(Levels.INTERMEDIATE);
        lift.runToPreset(Levels.INTERMEDIATE);
        state = Levels.LOCATING_TARGETS;
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
            case 1:
                stopIntake();
                return false;
            case -1:
                claw.eject();
                return true;
        }
        return true;
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
        extension.runToPosition(0);
        lift.runToPosition(0);
        state = Levels.HIGH_RUNG;
    }

    /**
     * <h1>Drop preload to the goat <u><b>DITA RAJEEV</b></u></h1>
     */
    public void preloadDropPreset() {
        arm.runToPreset(Levels.HIGH_RUNG);
        lift.runToPreset(Levels.HIGH_RUNG);
        state = Levels.LOW_BASKET;
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

    public Action outtakeSample(boolean action) {
        return claw.eject(true);
    }

    public void outtakeSpecimen() {
        lift.runToPosition(lift.getPos() - 10);
        new Thread(() -> {
            sleep(50);
            claw.eject();
        }).start();
    }

    public Action outtakeSpecimen(boolean action) {
        return new SequentialAction(
                new InstantAction(() -> lift.runToPosition(lift.getPos() - 10)),
                new SleepAction(0.15),
                claw.eject(true)
        );
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

    public Action smartOuttake(boolean action) {
        if (state == Levels.LOW_BASKET || state == Levels.HIGH_BASKET) {
            return new SequentialAction(
                    outtakeSample(true),
                    new InstantAction(() -> {
                        intermediatePreset();
                        state = Levels.INTERMEDIATE;
                    })
            );
        } else if (state == Levels.LOW_RUNG || state == Levels.HIGH_RUNG) {
            return new SequentialAction(
                    outtakeSpecimen(true),
                    new InstantAction(() -> {
                        intermediatePreset();
                        state = Levels.INTERMEDIATE;
                    })
            );
        }
        return new NullAction();
    }


    public Action primeClimb() {
        return new InstantAction(() -> {
                    climbWinch.up();
                    if (state == Levels.CLIMB_PRIMED) {
                        lift.runToPreset(Levels.INTERMEDIATE);
                        state = Levels.INTERMEDIATE;
                    } else {
                        lift.runToPreset(Levels.CLIMB_EXTENDED);
                        state = Levels.CLIMB_PRIMED;
                    }
                });
    }
    public Action startClimbL2(Gamepad gamepad) {
        return new SequentialAction(
                new InstantAction(() -> {
                    climbWinch.climb();
                    state = Levels.ASCENDING;
                }),
                commands.waitL2WinchClimbCompletion(gamepad)
        );
    }

    public Action startClimbL3() {
        return new InstantAction(() -> {
            lift.runToPreset(Levels.CLIMB_RETRACTED);
            state = Levels.CLIMB_L3;
        });
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

    public static double normalizeRadians(double angle) {
        angle = angle % (2 * Math.PI);
        if (angle < 0) {
            angle += 2 * Math.PI;
        }
        return angle;
    }
}