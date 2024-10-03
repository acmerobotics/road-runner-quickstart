package org.firstinspires.ftc.teamcode.New.SubSystems.Shawty.Java;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Tele", group = "Linear OpMode")
public class JohnTele extends LinearOpMode {
    //note: All gamepad inputs are not final, they are placeholders for the actual input or other booleans

    //                       HNG    BSK  CLP   SUB GND STA
    int[] verticalTargets = {5000, 4000, 3000, 100, 0, 0};
    //verttarget[0] controls hang to mannually lower so we don't hang on air

    //                       HNG BSK CLP SUB  GND  STA
    int[] horizontalTargets = {0, 0, 0, 1000, 1000, 0};
    //horiztarget[3] controls submersible reach, horiztarget[4] controls ground reach if we have to reach out further or less

    LinearSlides verticalSlides = new LinearSlides(hardwareMap);
    LinearSlides horizontalSlides = new LinearSlides(hardwareMap);
    Intake intake = new Intake(hardwareMap);
    Bucket bucket = new Bucket(hardwareMap);

    ElapsedTime elapsedTime = new ElapsedTime();

    boolean tranferring = false;
    double timeMarker;

    @Override
    public void runOpMode() {
        verticalSlides.resetValue = 0;
        horizontalSlides.resetValue = 0;

        state = RobotState.STATIONARY;
        update();

        waitForStart();

        elapsedTime.reset();

        while (opModeIsActive()){

            if(gamepad2.x) {
                state = RobotState.GROUND;
            }
            if(gamepad2.y) {
                state = RobotState.SUBMERSIBLE;
            }
            if(gamepad2.a) {
                state = RobotState.BASKET;
            }
            /* not needed until we have a clip mechanism
            if(gamepad2.b) {
                state = RobotState.CLIP;
            }
            */
            if (gamepad2.left_bumper && gamepad2.right_bumper){
                state = RobotState.HANG;
            }

            update();

            telemetry.addData("rightEncoder", verticalSlides.rightSlide.rightEncoder);
            telemetry.addData("leftEncoder", verticalSlides.leftSlide.leftEncoder);
            telemetry.update();

        }
    }
    enum RobotState {
        HANG,
        BASKET,
        CLIP,
        SUBMERSIBLE,
        GROUND,
        STATIONARY
    }

    RobotState state;

    private void update(){
        switch(state){
            case HANG:
                verticalSlides.state = LinearSlides.State.HANG;
                horizontalSlides.state = LinearSlides.State.HANG;
                intake.intakeState = Intake.IntakeState.STOPPED;
                intake.flipperState = Intake.FlipperState.STATIONARY;
                bucket.state = Bucket.State.STATIONARY;
                verticalTargets[0] += (int) -gamepad2.left_stick_y;

                break;
            case BASKET:
                verticalSlides.state = LinearSlides.State.BASKET;
                horizontalSlides.state = LinearSlides.State.BASKET;
                intake.intakeState = Intake.IntakeState.STOPPED;
                intake.flipperState = Intake.FlipperState.STATIONARY;
                bucket.state = Bucket.State.STATIONARY;


                if (gamepad2.x) {
                    tranferring = true;
                    timeMarker = elapsedTime.seconds();
                }
                if (tranferring) {
                    bucket.state = Bucket.State.DROPPING;
                    if ((elapsedTime.seconds() - timeMarker) >= 0.2) {
                        state = RobotState.STATIONARY;
                        tranferring = false;
                    }
                }
                break;
            case CLIP:
                verticalSlides.state = LinearSlides.State.CLIP;
                horizontalSlides.state = LinearSlides.State.CLIP;
                intake.intakeState = Intake.IntakeState.STOPPED;
                intake.flipperState = Intake.FlipperState.STATIONARY;
                bucket.state = Bucket.State.STATIONARY;
                break;
            case SUBMERSIBLE:
                verticalSlides.state = LinearSlides.State.SUBMERSIBLE;
                horizontalSlides.state = LinearSlides.State.SUBMERSIBLE;
                horizontalTargets[3] = (int) -gamepad2.right_stick_y;
                intake.intakeState = Intake.IntakeState.IN;
                intake.flipperState = Intake.FlipperState.COLLECTING;
                bucket.state = Bucket.State.STATIONARY;

                //would put if color sensor sees the opposite alliance color
                if(gamepad2.y && !tranferring){
                    intake.intakeState = Intake.IntakeState.OUT;
                }

                //if color sensor sees the right color
                if (gamepad2.a){
                    tranferring = true;
                    timeMarker = elapsedTime.seconds();
                }

                if (tranferring) {
                    intake.flipperState = Intake.FlipperState.TRANSFERRING;
                    if ((elapsedTime.seconds() - timeMarker) >= 0.1) {
                        intake.intakeState = Intake.IntakeState.OUT;
                        if ((elapsedTime.seconds() - timeMarker) >= 0.2) {
                            state = RobotState.STATIONARY;
                            tranferring = false;
                        }
                    }
                }
                break;
            case GROUND:
                verticalSlides.state = LinearSlides.State.GROUND;
                horizontalSlides.state = LinearSlides.State.GROUND;
                horizontalTargets[4] = (int) -gamepad2.right_stick_y;
                intake.intakeState = Intake.IntakeState.IN;
                intake.flipperState = Intake.FlipperState.COLLECTING;
                bucket.state = Bucket.State.STATIONARY;

                //would put if color sensor sees the opposite alliance color
                if(gamepad2.y && !tranferring){
                    intake.intakeState = Intake.IntakeState.OUT;
                }

                //if color sensor sees the right color
                if (gamepad2.a){
                    tranferring = true;
                    timeMarker = elapsedTime.seconds();
                }

                if (tranferring) {
                    intake.flipperState = Intake.FlipperState.TRANSFERRING;
                    if ((elapsedTime.seconds() - timeMarker) >= 0.1) {
                        intake.intakeState = Intake.IntakeState.OUT;
                        if ((elapsedTime.seconds() - timeMarker) >= 0.2) {
                            state = RobotState.STATIONARY;
                            tranferring = false;
                        }
                    }
                }
                break;
            case STATIONARY:
                verticalSlides.state = LinearSlides.State.STATIONARY;
                horizontalSlides.state = LinearSlides.State.STATIONARY;
                intake.intakeState = Intake.IntakeState.STOPPED;
                intake.flipperState = Intake.FlipperState.STATIONARY;
                bucket.state = Bucket.State.STATIONARY;
                break;
        }

        verticalSlides.update(verticalTargets);
        horizontalSlides.update(horizontalTargets);
        intake.update();
        bucket.update();
        TelemetryUpdate();
    }

    private void TelemetryUpdate(){
        telemetry.addData("Robot State", state);
        telemetry.addData("Vertical Slides State", verticalSlides.state);
        telemetry.addData("Horizontal Slides State", horizontalSlides.state);
        telemetry.addData("Intake State", intake.intakeState);
        telemetry.addData("Intake Flipper State", intake.flipperState);
        telemetry.addData("Bucket State", bucket.state);

        telemetry.update();
    }
}
