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
    public void loop(AIMPad aimpad, AIMPad aimpad2) {
        hand.loop(aimpad);
        wrist.loop(aimpad);
        elbow.loop(aimpad);
    }

    public void resetOpen() {
        hand.open();
        wrist.flexUp();
        wrist.rotateLeft();
        elbow.forward();
    }

    public void resetClosed() {
        hand.close();
        wrist.flexUp();
        wrist.rotateLeft();
        elbow.forward();
    }

    public void resetAvoid() {
        hand.close();
        wrist.flexNeutral();
        wrist.rotateCenter();
        elbow.backward();
    }

    public void resetAvoidNeutral() {
        hand.open();
        wrist.flexNeutral();
        wrist.rotateCenter();
        elbow.middle();
    }

    public void neutral() {
        hand.open();
        wrist.flexNeutral();
        wrist.rotateCenter();
        elbow.middle();
    }

    public void neutralClosed() {
        hand.close();
        wrist.flexNeutral();
        wrist.rotateCenter();
        elbow.middle();
    }

    public void upClosed() {
        hand.close();
        wrist.flexUp();
        wrist.rotateCenter();
        elbow.middle();
    }

    public void searchingDownOpen() {
        hand.open();
        wrist.flexDown();
        wrist.rotateCenter();
        elbow.backward();
    }

    public void searchingDownLeftTiltOpen() {
        hand.open();
        wrist.flexDown();
        wrist.rotateLeft();
        elbow.backward();
    }

    public void searchingDownRightTiltOpen() {
        hand.open();
        wrist.flexDown();
        wrist.rotateRight();
        elbow.backward();
    }

    public void searchingDownLeftTiltClosed() {
        hand.close();
        wrist.flexDown();
        wrist.rotateLeft();
        elbow.backward();
    }

    public void searchingDownRightTiltClosed() {
        hand.close();
        wrist.flexDown();
        wrist.rotateRight();
        elbow.backward();
    }

    public void searchingDownClosed() {
        hand.close();
        wrist.flexDown();
        wrist.rotateCenter();
        elbow.backward();
    }

    public void custom(double handPosition, double wristFlexPosition, double wristRotatePosition, double elbowPosition) {
        hand.custom(handPosition);
        wrist.flexCustom(wristFlexPosition);
        wrist.rotateCustom(wristRotatePosition);
        elbow.custom(elbowPosition);
    }
}
