package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.multiaxisarm.MultiAxisArm;

public class ScoringAssembly extends Mechanism {

    boolean disableArm = false;

    public MultiAxisArm multiAxisArm;
    public Pivot pivot;
    public Slides slides;

    @Override
    public void init(HardwareMap hwMap) {
        multiAxisArm = new MultiAxisArm();
        multiAxisArm.init(hwMap);
        slides = new Slides();
        slides.init(hwMap);
        pivot = new Pivot(slides);
        pivot.init(hwMap);
    }

    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        slides.loop(aimpad, aimpad2);
        if (!disableArm) {
            multiAxisArm.loop(aimpad, aimpad2);
            pivot.loop(aimpad, aimpad2);
        }
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        pivot.telemetry(telemetry);
        slides.telemetry(telemetry);
    }

    public void reset() {
        multiAxisArm.resetOpen();
        pivot.setPivotPosition(Pivot.PivotAngle.SCORE);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public void resetSpecimen() {
        multiAxisArm.specimenPickup();
        pivot.setPivotPosition(Pivot.PivotAngle.SPECIMEN_PICKUP);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public void resetAuto() {
        multiAxisArm.neutral();
        pivot.setPivotPosition(Pivot.PivotAngle.START_MORE);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET_MORE);
    }

    public void resetAvoid() {
        multiAxisArm.down();
        pivot.setPivotPosition(Pivot.PivotAngle.HIGH_BUCKET_RESET);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public void setPickupReset() {
        multiAxisArm.down();
        pivot.setPivotPosition(Pivot.PivotAngle.PICKUP);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }
    public void setPickupResetNeutralClosed() {
        multiAxisArm.neutralClosed();
        pivot.setPivotPosition(Pivot.PivotAngle.PICKUP);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public void setPickupResetClamped() {
        multiAxisArm.neutralClosed();
        pivot.setPivotPosition(Pivot.PivotAngle.PICKUP);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public void setScoringResetClamped() {
        multiAxisArm.upClosed();
        pivot.setPivotPosition(Pivot.PivotAngle.NEW_SCORE);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public void setSpecimenClamped() {
        multiAxisArm.upClosed();
        pivot.setPivotPosition(Pivot.PivotAngle.SCORE);
        slides.setSlidesPosition(Slides.SlidesExtension.HIGH_SPECIMEN);
    }

    public void setSpecimenClampedAUTO() {
        multiAxisArm.upClosed();
        pivot.setPivotPosition(Pivot.PivotAngle.SPECIMEN_PICKUP);
        slides.setSlidesPosition(Slides.SlidesExtension.HIGH_SPECIMEN_AUTO);
    }

    public void totalFix() {
        multiAxisArm.neutral();
        slides.setSlidesPosition(Slides.SlidesExtension.RESET_MORE);
        pivot.setPivotPosition(Pivot.PivotAngle.START_MORE);
    }

    //====================================================================================================
    // Hanging presets
    //====================================================================================================

    public void setHangStart() {
        multiAxisArm.hang();
        pivot.setPivotPosition(Pivot.PivotAngle.START);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public void setLowHangExtended() {
        multiAxisArm.hang();
        pivot.setPivotPosition(Pivot.PivotAngle.START);
        slides.setSlidesPosition(Slides.SlidesExtension.LOW_HANG);
    }

    public void setLowHangRetracted() {
        disableArm = true;
        multiAxisArm.hang();
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pivot.setPivotAtPower(0);
//        pivot.setPivotPosition(Pivot.PivotAngle.START);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public void setLowHangClip() {
        disableArm = true;
        multiAxisArm.hang();
        pivot.setPivotPosition(Pivot.PivotAngle.CLIP_ON);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public void setHighHangOff() {
        multiAxisArm.hang();
        pivot.setPivotPosition(Pivot.PivotAngle.HANG_OFF);
        slides.setSlidesPosition(Slides.SlidesExtension.HIGH_HANG);
    }

    public void setHighHangOn() {
        multiAxisArm.hang();
        pivot.setPivotPosition(Pivot.PivotAngle.NEW_SCORE);
        slides.setSlidesPosition(Slides.SlidesExtension.HIGH_HANG);
    }

    public void setHighHangRetracted() {
        multiAxisArm.hang();
        pivot.setPivotAtPower(0);
//        pivot.setPivotPosition(Pivot.PivotAngle.PICKUP);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public void setHighHangFinal() {
        multiAxisArm.hang();
        pivot.setPivotPosition(Pivot.PivotAngle.NEW_SCORE);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public boolean areMotorsAtTarget() {
        return pivot.isAtTargetAngle() && slides.isAtTargetPosition();
    }

    public boolean areMotorsAtTargetPresets() {
        return pivot.isAtTargetPreset() && slides.isAtTargetPreset();
    }
}
