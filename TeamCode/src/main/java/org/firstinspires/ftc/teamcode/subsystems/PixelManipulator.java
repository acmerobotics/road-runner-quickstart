package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class PixelManipulator extends Mechanism {

    public Arm arm;
    public Claw claw;
    public Intake intake;
    public PIDSlides slides;


    enum ScoringState {
        PICKING_UP, PIXELSLOADED_ONE, PIXELSLOADED_TWO, POSITIONING, RELEASINGLEFT, RELEASINGRIGHT, RESETTING_STAGE_ONE, RESETTING_STAGE_TWO, FULL_MANUAL
    }

    ScoringState activeScoringState = ScoringState.RESETTING_STAGE_TWO;

    @Override
    public void init(HardwareMap hwMap) {
        intake = new Intake();
        claw = new Claw();
        arm = new Arm();
        slides = new PIDSlides();

        intake.init(hwMap);
        claw.init(hwMap);
        arm.init(hwMap);
        slides.init(hwMap);
        arm.retract();
    }

    @Override
    public void loop(Gamepad gamepad, Gamepad gamepad2) {
        intake.loop(gamepad);
        claw.loop(gamepad);
        arm.loop(gamepad);
        slides.loop(gamepad);

        boolean isPixelsLoaded = claw.isLeftClamped && claw.isRightClamped;

        boolean isReleasable = !slides.isSpeeding();

        boolean manualOverride = gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right || gamepad2.dpad_up;


        if (manualOverride) {
            setActiveScoringState(ScoringState.FULL_MANUAL);
        }

        switch (activeScoringState) {
            case PICKING_UP:
                if (gamepad2.y) {
                    intake.outtake(1);
                } else {
                    intake.intake(1);
                }
                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
                    claw.clampServo(claw.leftProng);
                } else if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
                    claw.clampServo(claw.rightProng);
                } else if (gamepad2.left_bumper) {
                    claw.releaseLeftServo(claw.leftProng);
                } else if (gamepad2.right_bumper) {
                    claw.releaseServo(claw.rightProng);
                }
                if (isPixelsLoaded && gamepad2.a) {
                    setActiveScoringState(ScoringState.PIXELSLOADED_ONE);
                }
                break;

            case PIXELSLOADED_ONE:
                intake.stop();
                arm.safeRetract();
                slides.update(PIDSlides.SAFE_EXTENSION_POS);
                if (slides.isAtTargetPosition()) {
                    setActiveScoringState(ScoringState.PIXELSLOADED_TWO);
                }
                break;

            case PIXELSLOADED_TWO:
                arm.extend();
                if (arm.isExtended && gamepad2.a) {
                    setActiveScoringState(ScoringState.POSITIONING);
                }
                break;


            case POSITIONING:
                if (Math.abs(gamepad2.left_stick_y) > GamepadSettings.GP2_STICK_DEADZONE) {
                    slides.setPower(gamepad2.left_stick_y);
                } else {
                    slides.holdPosition();
                }

                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGLEFT);
                } else if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGRIGHT);
                }
                break;

            case RELEASINGLEFT:
                claw.releaseLeftServo(claw.leftProng);
                if (claw.isLeftReleased && claw.isRightReleased) {
                    activeScoringState = ScoringState.RESETTING_STAGE_ONE;
                }
                if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGRIGHT);
                }
                break;

            case RELEASINGRIGHT:
                claw.releaseServo(claw.rightProng);
                if (claw.isLeftReleased && claw.isRightReleased) {
                    activeScoringState = ScoringState.RESETTING_STAGE_ONE;
                }
                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isReleasable) {
                    setActiveScoringState(ScoringState.RELEASINGLEFT);
                }
                break;

            case RESETTING_STAGE_ONE:
                slides.update(PIDSlides.SAFE_RETRACTION_POS);
                if (slides.isAtTargetPosition()) {
                    arm.safeRetract();
                    if (gamepad2.a && gamepad2.x && gamepad2.y) {
                        setActiveScoringState(ScoringState.RESETTING_STAGE_TWO);
                    }
                }
                break;

            case RESETTING_STAGE_TWO:
                slides.update(PIDSlides.RESET_POS);
                claw.releaseLeftServo(claw.leftProng);
                claw.releaseServo(claw.rightProng);
                if (slides.isAtTargetPosition()) {
                    arm.retract();
                    setActiveScoringState(ScoringState.PICKING_UP);
                }
                break;
            case FULL_MANUAL:
                if (Math.abs(gamepad2.left_stick_y) > GamepadSettings.GP2_STICK_DEADZONE) {
                    slides.setPower(gamepad2.left_stick_y);
                } else {
                    slides.holdPosition();
                }

                if (gamepad2.left_bumper) {
                    claw.clampServo(claw.leftProng);
                } else if (gamepad2.right_bumper) {
                    claw.clampServo(claw.rightProng);
                } else if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
                    claw.releaseLeftServo(claw.leftProng);
                } else if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
                    claw.releaseServo(claw.rightProng);
                }

                if (gamepad2.a) {
                    arm.extend();
                } else if (gamepad2.y) {
                    arm.retract();
                } else if (Math.abs(gamepad2.right_stick_y) > GamepadSettings.GP2_TRIGGER_DEADZONE) {
                    arm.setPower(gamepad2.right_stick_y * .002);
                }

                if (gamepad2.x) {
                    intake.intake(1);
                } else if (gamepad2.b) {
                    intake.outtake(1);
                }
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        intake.telemetry(telemetry);
        claw.telemetry(telemetry);
        arm.telemetry(telemetry);
        slides.telemetry(telemetry);
        telemetry.addData("Active Scoring State", activeScoringState);
    }

    public void setActiveScoringState(ScoringState state) {
        activeScoringState = state;
    }
}