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
        pivot.loop(aimpad);
        slides.loop(aimpad);
        multiAxisArm.loop(aimpad);
    }

    public void reset() {
        multiAxisArm.resetOpen();
        pivot.setPivotPosition(Pivot.PivotPosition.SCORE);
        slides.setSlidesPosition(Slides.SlidesPosition.RESET);
    }

    public void setPickupResetNeutral() {
        multiAxisArm.neutral();
        pivot.setPivotPosition(Pivot.PivotPosition.PICKUP);
        slides.setSlidesPosition(Slides.SlidesPosition.RESET);
    }

    public void setPickupResetClamped() {
        multiAxisArm.neutralClosed();
        pivot.setPivotPosition(Pivot.PivotPosition.PICKUP);
        slides.setSlidesPosition(Slides.SlidesPosition.RESET);
    }

    public void setScoringResetClamped() {
        multiAxisArm.neutralClosed();
        pivot.setPivotPosition(Pivot.PivotPosition.SCORE);
        slides.setSlidesPosition(Slides.SlidesPosition.RESET);
    }

    public void setScoringLowBucketClamped() {
        multiAxisArm.neutralClosed();
        pivot.setPivotPosition(Pivot.PivotPosition.SCORE);
        slides.setSlidesPosition(Slides.SlidesPosition.LOW_BUCKET);
    }

    public boolean areMotorsAtTarget() {
        return pivot.isAtTargetPosition() && slides.isAtTargetPosition();
    }
}
