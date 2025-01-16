package org.firstinspires.ftc.teamcode.subsystems.multiaxisarm;

import com.aimrobotics.aimlib.gamepad.AIMPad;
import com.aimrobotics.aimlib.util.Mechanism;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MultiAxisArm extends Mechanism {

    public Hand hand;
    public Wrist wrist;
    public Elbow elbow;

    @Override
    public void init(HardwareMap hwMap) {
        hand = new Hand();
        wrist = new Wrist();
        elbow = new Elbow();

        hand.init(hwMap);
        wrist.init(hwMap);
        elbow.init(hwMap);
    }

    @Override
    public void loop(AIMPad aimpad) {
        hand.loop(aimpad);
        wrist.loop(aimpad);
        elbow.loop(aimpad);
    }

    public void resetOpen() {
        hand.open();
        wrist.flexUp();
        wrist.rotateLeft();
        elbow.up();
    }

    public void resetClosed() {
        hand.close();
        wrist.flexUp();
        wrist.rotateLeft();
        elbow.up();
    }

    public void resetAvoid() {
        hand.open();
        wrist.flexDown();
        wrist.rotateCenter();
        elbow.avoid();
    }

    public void searchingNeutral() {
        hand.open();
        wrist.flexNeutral();
        wrist.rotateCenter();
        elbow.down();
    }

    public void searchingDownOpen() {
        hand.open();
        wrist.flexDown();
        wrist.rotateCenter();
        elbow.down();
    }

    public void searchingDownLeftTiltOpen() {
        hand.open();
        wrist.flexDown();
        wrist.rotateLeft();
        elbow.down();
    }

    public void searchingDownRightTiltOpen() {
        hand.open();
        wrist.flexDown();
        wrist.rotateRight();
        elbow.down();
    }

    public void searchingDownLeftTiltClosed() {
        hand.close();
        wrist.flexDown();
        wrist.rotateLeft();
        elbow.down();
    }

    public void searchingDownRightTiltClosed() {
        hand.close();
        wrist.flexDown();
        wrist.rotateRight();
        elbow.down();
    }

    public void searchingDownClosed() {
        hand.close();
        wrist.flexDown();
        wrist.rotateCenter();
        elbow.down();
    }
}
