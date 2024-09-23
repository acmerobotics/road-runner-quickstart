package org.firstinspires.ftc.teamcode;

// IMPORT SUBSYSTEMS

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.commands.CommandMaster;
import org.firstinspires.ftc.teamcode.subsystems.endEffector.EndEffector;
import org.firstinspires.ftc.teamcode.subsystems.extension.Extension;
import org.firstinspires.ftc.teamcode.subsystems.lift.Lift;
import org.firstinspires.ftc.teamcode.subsystems.v4b.V4B;
import org.firstinspires.ftc.teamcode.util.Component;
import org.firstinspires.ftc.teamcode.util.ContinuousServo;
import org.firstinspires.ftc.teamcode.util.Motor;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.StepperServo;

public class Robot {

    // SUBSYSTEM DECLARATIONS
    public Component[] components;
    public MecanumDrive drive;
    public Lift lift;
    public Extension extension;
    public V4B v4b;
    public EndEffector endEffector;

    public CommandMaster commands;
    public HardwareMap hardwareMap;

    // STATE VARS
    boolean auton;

    Motor backLeft;
    Motor backRight;
    Motor frontLeft;
    Motor frontRight;


    public Robot(HardwareMap map, boolean auton){
        this.auton = auton;

        this.drive = new MecanumDrive(map, new Pose2d(0,0,0));

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
        this.v4b = new V4B((StepperServo) components[8]);
        this.endEffector = new EndEffector((ContinuousServo) components[10], (ContinuousServo) components[11], (StepperServo) components[9], colorSensor);

        this.commands = new CommandMaster(this);
        this.hardwareMap = map;

        backLeft = (Motor) components[0];
        backRight = (Motor) components[1];
        frontLeft = (Motor) components[2];
        frontRight = (Motor) components[3];
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
}