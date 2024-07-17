package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.StateMachine;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    enum RobotStates
    {
        PIXELS_IN_STORAGE_SLIDES_EXTENDED,
        PIXELS_IN_STORAGE_SLIDES_RETRACTED,
        OUTTAKE_READY,
        WAITING_FOR_CLAW,
        LIFT_UP,
        WAITING_FOR_REVERSE_INTAKE,
        LIFT_GOING_DOWN,
    }
    RobotStates lastState = RobotStates.OUTTAKE_READY;
    @Override
    public void runOpMode() throws InterruptedException {
        StateMachine machine = new StateMachineBuilder()
                .state(RobotStates.PIXELS_IN_STORAGE_SLIDES_EXTENDED)
                .onEnter( () -> {
                    intake.spit();
                    slides.retract();
                })
                .transition( () -> slides.currentState == retracted, RobotStates.PIXELS_IN_STORAGE_SLIDES_RETRACTED)

                .state(RobotStates.PIXELS_IN_STORAGE_SLIDES_RETRACTED)
                .onEnter( () -> {
                    latch.open();
                    outtake4bar.goToIntake();
                    outtakeJoint.goToIntake();
                })
                .onExit( () -> {
                    outtakeClaw.closeLeft();
                    outtakeClaw.closeRight();
                })
                .transitionTimed(1, RobotStates.WAITING_FOR_CLAW)

                .state(RobotStates.WAITING_FOR_CLAW)
                .onExit( () -> {
                    if(claw.isEmpty() && lift.currentState == UP) {
                        lift.currentState = GOING_DOWN_PID;
                    }
                })
                .transitionTimed(0.2, RobotStates.OUTTAKE_READY)

                .state(RobotStates.OUTTAKE_READY)
                .onEnter( () -> {
                    outtake4bar.goToReady();
                    outtakeJoint.goToReady();
                    outtakeRotation.goToLevel();
                })
                .transition(lift.currentState == UP, RobotStates.LIFT_UP)
                .transition(slides.currentState == EXTENDED && storage.currentState == full, RobotStates.WAITING_FOR_REVERSE_INTAKE)

                .state(RobotStates.WAITING_FOR_REVERSE_INTAKE)
                .transitionTimed(0.2, RobotStates.PIXELS_IN_STORAGE_SLIDES_EXTENDED)

                .state(RobotStates.LIFT_UP)
                .onEnter( () -> {
                    outtake4bar.goToOuttake();
                    outtakeJoint.goToOuttake();
                })
                .transition(claw.isEmpty() || lift.currentState != UP, RobotStates.WAITING_FOR_CLAW)

                .build();
    }
}
