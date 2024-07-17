package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachineBuilder;

<<<<<<< Updated upstream
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Utils.StickyGamepad;
import org.firstinspires.ftc.teamcode.teamCode.Classes.ClawController;
=======
>>>>>>> Stashed changes
import org.firstinspires.ftc.teamcode.teamCode.Classes.IntakeController;
import org.firstinspires.ftc.teamcode.teamCode.Classes.LiftController;
import org.firstinspires.ftc.teamcode.teamCode.Classes.Outtake4Bar;
import org.firstinspires.ftc.teamcode.teamCode.Classes.OuttakeJoint;
import org.firstinspires.ftc.teamcode.teamCode.Classes.OuttakeRotation;
import org.firstinspires.ftc.teamcode.teamCode.Classes.ExtendoControllerPID;
import org.firstinspires.ftc.teamcode.teamCode.Classes.Storage;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    LiftController lift;
    OuttakeRotation outtakeRotation;
    Outtake4Bar outtake4Bar;
    OuttakeJoint outtakeJoint;
    IntakeController intake;
    Storage storage;
    ExtendoControllerPID slides;
<<<<<<< Updated upstream
    ClawController claw;
    SampleMecanumDrive drive;
    StickyGamepad sticky1, sticky2;

=======
>>>>>>> Stashed changes

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
                .onEnter(() -> {
                    intake.reverse();
                    if(slides.currentState != ExtendoControllerPID.States.RETRACTED)
                        slides.retract();
                })
                .transition(() -> slides.currentState == ExtendoControllerPID.States.RETRACTED, RobotStates.PIXELS_IN_STORAGE_SLIDES_RETRACTED)

                .state(RobotStates.PIXELS_IN_STORAGE_SLIDES_RETRACTED)
                .onEnter(() -> {
                    intake.turnOff();
                    storage.open();
                    outtake4Bar.goToIntake();
                    outtakeJoint.goToIntake();
                })
                .onExit(() -> {
                    claw.goToPlace();
                })
                .transitionTimed(1, RobotStates.WAITING_FOR_CLAW)

                .state(RobotStates.WAITING_FOR_CLAW)
                .onExit(() -> {
                    if (claw.isEmpty() && lift.currentState == LiftController.States.UP) {
                        lift.currentState = LiftController.States.GOING_DOWN_PID;
                    }
                })
                .transitionTimed(0.2, RobotStates.OUTTAKE_READY)

                .state(RobotStates.OUTTAKE_READY)
                .onEnter(() -> {
                    outtake4Bar.goToReady();
                    outtakeJoint.goToReady();
                    outtakeRotation.goToLevel();
                })
                .transition( () -> lift.currentState == LiftController.States.UP, RobotStates.LIFT_UP)
                .transition( () -> storage.currentState == Storage.State.FULL, RobotStates.WAITING_FOR_REVERSE_INTAKE)

                .state(RobotStates.WAITING_FOR_REVERSE_INTAKE)
                .transitionTimed(0.2, RobotStates.PIXELS_IN_STORAGE_SLIDES_EXTENDED)

                .state(RobotStates.LIFT_UP)
                .onEnter(() -> {
                    outtake4Bar.goToDrop();
                    outtakeJoint.goToDrop();
                })
                .transition( () -> claw.isEmpty() || lift.currentState != LiftController.States.UP, RobotStates.WAITING_FOR_CLAW)

                .build();

        waitForStart();

        machine.setState(RobotStates.OUTTAKE_READY);
        machine.start();

        while(opModeIsActive()) {




            sticky1.update();
            sticky2.update();
            drive.RobotCentric(gamepad1);
            machine.update();
        }
    }
}
