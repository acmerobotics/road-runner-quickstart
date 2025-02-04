package org.firstinspires.ftc.teamcode.subsystems;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.multiaxisarm.MultiAxisArm;

public class ScoringAssembly extends Mechanism {

    public MultiAxisArm multiAxisArm;
    public Pivot pivot;
    public Slides slides;

    boolean isReset = false;

    @Override
    public void init(HardwareMap hwMap) {
        multiAxisArm = new MultiAxisArm();
        multiAxisArm.init(hwMap);
        pivot = new Pivot();
        pivot.init(hwMap);
        slides = new Slides();
        slides.init(hwMap);
    }

    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        pivot.loop(aimpad, aimpad2);
        slides.loop(aimpad, aimpad2);
        multiAxisArm.loop(aimpad, aimpad2);
        pivot.setIsFreeMovementEnabled(slides.isPivotEnabled);
    }

    public void reset() {
        multiAxisArm.resetOpen();
        pivot.setPivotPosition(Pivot.PivotAngle.SCORE);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public void setPickupResetNeutral() {
        multiAxisArm.neutral();
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
        pivot.setPivotPosition(Pivot.PivotAngle.SCORE);
        slides.setSlidesPosition(Slides.SlidesExtension.RESET);
    }

    public void setScoringLowBucketClamped() {
        multiAxisArm.neutralClosed();
        pivot.setPivotPosition(Pivot.PivotAngle.SCORE);
        slides.setSlidesPosition(Slides.SlidesExtension.LOW_BUCKET);
    }

    public boolean areMotorsAtTarget() {
        return pivot.isAtTargetAngle() && slides.isAtTargetPosition();
    }

    public boolean areMotorsAtTargetPresets() {
        return  pivot.isAtTargetPreset() && slides.isAtTargetPreset();
    }
}
