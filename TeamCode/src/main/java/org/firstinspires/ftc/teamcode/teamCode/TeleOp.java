package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachineBuilder;
import com.sfdev.assembly.state.*;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.teamcode.configs.StateMachineTest;
import org.firstinspires.ftc.teamcode.teamCode.Classes.IntakeController;
import org.firstinspires.ftc.teamcode.teamCode.Classes.LiftController;
import org.firstinspires.ftc.teamcode.teamCode.Classes.Outtake4Bar;
import org.firstinspires.ftc.teamcode.teamCode.Classes.OuttakeJoint;
import org.firstinspires.ftc.teamcode.teamCode.Classes.OuttakeRotation;
import org.firstinspires.ftc.teamcode.teamCode.Classes.SlidesControllerPID;
import org.firstinspires.ftc.teamcode.teamCode.Classes.Storage;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    LiftController lift;
    OuttakeRotation outtakeRotation;
    Outtake4Bar outtake4Bar;
    OuttakeJoint outtakeJoint;
    IntakeController intake;
    Storage storage;
    SlidesControllerPID slides;

    enum RobotStates
    {
        PIXELS_IN_STORAGE_SLIDES_EXTENDED,
        PIXELS_IN_STORAGE_SLIDES_RETRACTED,
        OUTTAKE_READY,
        WAITING_FOR_CLAW,
        LIFT_UP,
        WAITING_FOR_REVERSE_INTAKE,
    }

//    public static StateMachine getStateMachine() {
//        return new StateMachineBuilder

//    }


    RobotStates lastState = RobotStates.OUTTAKE_READY;
    @Override
    public void runOpMode() throws InterruptedException {

        com.sfdev.assembly.state.StateMachine machine = new StateMachineBuilder()
                .state(RobotStates.PIXELS_IN_STORAGE_SLIDES_EXTENDED)
                .onExit(() -> {
                    telemetry.addData("", "");
                })
                .onEnter(() -> {
                    intake.reverse();
                    slides.retract();
                })
                .transition(() -> slides.currentState == SlidesControllerPID.States.RETRACTED, RobotStates.PIXELS_IN_STORAGE_SLIDES_RETRACTED)

                .state(RobotStates.PIXELS_IN_STORAGE_SLIDES_RETRACTED)
                .onEnter(() -> {
                    intake.turnOff();
                    storage.open();
                    outtake4Bar.goToIntake();
                    outtakeJoint.goToIntake();
                })
                .onExit(() -> {
                    outtakeClaw.closeLeft();
                    outtakeClaw.closeRight();
                })
                .transitionTimed(1, RobotStates.WAITING_FOR_CLAW)

                .state(RobotStates.WAITING_FOR_CLAW)
                .onExit(() -> {
                    if (claw.isEmpty() && lift.currentState == UP) {
                        lift.currentState = GOING_DOWN_PID;
                    }
                })
                .transitionTimed(0.2, RobotStates.OUTTAKE_READY)

                .state(RobotStates.OUTTAKE_READY)
                .onEnter(() -> {
                    outtake4Bar.goToReady();
                    outtakeJoint.goToReady();
                    outtakeRotation.goToLevel();
                })
                .transition(lift.currentState == UP, RobotStates.LIFT_UP)
                .transition(slides.currentState == EXTENDED && storage.currentState == full, RobotStates.WAITING_FOR_REVERSE_INTAKE)

                .state(RobotStates.WAITING_FOR_REVERSE_INTAKE)
                .transitionTimed(0.2, RobotStates.PIXELS_IN_STORAGE_SLIDES_EXTENDED)

                .state(RobotStates.LIFT_UP)
                .onEnter(() -> {
                    outtake4Bar.goToDrop();
                    outtakeJoint.goToDrop();
                })
                .transition(claw.isEmpty() || lift.currentState != UP, RobotStates.WAITING_FOR_CLAW)

                .build();


//        StateMachine machine = getStateMachine();

        waitForStart();

        machine.start();

        while(opModeIsActive()) {
            machine.update();
        }
    }
}
