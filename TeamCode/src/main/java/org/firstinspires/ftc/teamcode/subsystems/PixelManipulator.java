package org.firstinspires.ftc.teamcode.subsystems;

import com.ThermalEquilibrium.homeostasis.Utils.WPILibMotionProfile;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.developmental.PIDSlides;
import org.firstinspires.ftc.teamcode.subsystems.settings.GamepadSettings;
import org.firstinspires.ftc.teamcode.util.Mechanism;

public class PixelManipulator extends Mechanism {

    Arm arm;
    Claw claw;
    Intake intake;
    PIDSlides slides;


    enum ScoringState {
        PICKINGUP, PIXELSLOADED, POSITIONING, RELEASINGLEFT, RELEASINGRIGHT, RESETTING_STAGE_ONE, RESETTING_STAGE_TWO
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
    }

    @Override
    public void loop(Gamepad gamepad, Gamepad gamepad2) {
        intake.loop(gamepad);
        claw.loop(gamepad);
        arm.loop(gamepad);
        slides.loop(gamepad);

        boolean isPixelsLoaded = claw.isLeftClamped && claw.isRightClamped;

        boolean isRelseasable = claw.isRotatorInPosition && !slides.isSpeeding();

        switch (activeScoringState) {
            case PICKINGUP:
                intake.intake(1);
                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
                    claw.clampServo(claw.leftProng);
                } else if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE) {
                    claw.clampServo(claw.leftProng);
                }
                if (isPixelsLoaded && gamepad2.a) {
                    setActiveScoringState(ScoringState.PIXELSLOADED);
                }
                break;

            case PIXELSLOADED:
                intake.stop();
                slides.update(PIDSlides.SAFE_EXTENSION_POS);
                if (slides.isAtTargetPosition()) {
                    arm.extend();
                    if (arm.isExtended && gamepad2.a) {
                        setActiveScoringState(ScoringState.POSITIONING);
                    }
                }
                break;


            case POSITIONING:
                if (Math.abs(gamepad2.left_stick_y) > GamepadSettings.GP2_STICK_DEADZONE) {
                    slides.setPower(gamepad2.left_stick_y);
                } else {
                    slides.holdPosition();
                }

                if (gamepad2.x) {
                    claw.setActiveTiltState(Claw.TiltState.LEFT);
                } else if (gamepad2.y) {
                    claw.setActiveTiltState(Claw.TiltState.CENTER);
                } else if (gamepad2.b) {
                    claw.setActiveTiltState(Claw.TiltState.RIGHT);
                }
                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isRelseasable) {
                    setActiveScoringState(ScoringState.RELEASINGLEFT);
                } else if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isRelseasable) {
                    setActiveScoringState(ScoringState.RELEASINGRIGHT);
                }
                break;

            case RELEASINGLEFT:
                claw.releaseServo(claw.leftProng);
                if (claw.isLeftReleased && claw.isRightReleased) {
                    activeScoringState = ScoringState.RESETTING_STAGE_ONE;
                }
                if (gamepad2.right_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isRelseasable) {
                    setActiveScoringState(ScoringState.RELEASINGRIGHT);
                }
                break;

            case RELEASINGRIGHT:
                claw.releaseServo(claw.rightProng);
                if (claw.isLeftReleased && claw.isRightReleased) {
                    activeScoringState = ScoringState.RESETTING_STAGE_ONE;
                }
                if (gamepad2.left_trigger > GamepadSettings.GP2_TRIGGER_DEADZONE && isRelseasable) {
                    setActiveScoringState(ScoringState.RELEASINGLEFT);
                }
                break;

            case RESETTING_STAGE_ONE:
                claw.setActiveTiltState(Claw.TiltState.CENTER);
                slides.update(PIDSlides.SAFE_RETRACTION_POS);
                if (slides.isAtTargetPosition() && claw.isRotatorInPosition && gamepad.a) {
                    setActiveScoringState(ScoringState.RESETTING_STAGE_TWO);
                }
                break;

            case RESETTING_STAGE_TWO:
                arm.retract();
                if (arm.isRetracted && gamepad2.a) {
                    slides.resetSlidesPosition();
                    if (slides.isAtTargetPosition()) {
                        setActiveScoringState(ScoringState.PICKINGUP);
                    }
                }
        }
    }

    public void setActiveScoringState(ScoringState state) {
        activeScoringState = state;
    }
}