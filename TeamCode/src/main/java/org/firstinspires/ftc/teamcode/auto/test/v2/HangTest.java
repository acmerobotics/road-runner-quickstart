package org.firstinspires.ftc.teamcode.auto.test.v2;


//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.robotv2.LeftActuator;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.LeftActuatorServo;

import org.firstinspires.ftc.teamcode.mechanisms.robotv2.RightActuator;
import org.firstinspires.ftc.teamcode.mechanisms.robotv2.RightActuatorServo;
//import org.firstinspires.ftc.teamcode.mechanisms.Hang;

@Autonomous(name = "Hang Test", group = "Test")
public class HangTest extends LinearOpMode {



    @Override
    public void runOpMode() {
        LeftActuatorServo leftServo = new LeftActuatorServo(hardwareMap);
        LeftActuator leftActuator = new LeftActuator(hardwareMap);
        RightActuatorServo rightServo = new RightActuatorServo(hardwareMap);
        RightActuator rightActuator = new RightActuator(hardwareMap);

//        leftActuator.reset();
//        leftActuator.up();
        leftActuator.motor.setTargetPosition(LeftActuator.ACTUATOR_UP);
        leftActuator.motor.setVelocity(3000);
        leftActuator.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while(!isStopRequested() && !opModeIsActive()) {
        }
        waitForStart();

        if (isStopRequested()) return;



//        rightActuator.up();
//
//        Actions.runBlocking(
//            new SequentialAction(
//                hang.HangUpAction()
////                new SleepAction(4),
////                leftServo.setVerticalAction(),
////                rightServo.setVerticalAction(),
////                new SleepAction(4), // sleep
////                hang.HangDownAction(),
////                new SleepAction(4),
////                hang.HangUpAction(),
////                leftServo.setHorizontalAction(),
////                rightServo.setHorizontalAction()
//            ));
    } // runOpMode



}

